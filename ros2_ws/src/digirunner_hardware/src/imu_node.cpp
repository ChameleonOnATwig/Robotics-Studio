// Copyright 2026 Digi-Runner team. MIT License.
//
// imu_node: reads the GY-521 (MPU-6050) over I2C and publishes imu/data (sensor_msgs/Imu) in the
// robot's REP 103 frame (x forward, y left, z up). Pitch and roll come from a complementary
// filter; yaw is always 0 (no magnetometer).
//   pitch > 0: leaning forward    roll > 0: leaning right    (same signs as code/imu.py)
// Service imu/calibrate (std_srvs/Trigger): re-measure the gyro bias while the robot stands
// still. It replies once the samples are in (~2 s), without blocking the publish loop.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "digirunner_hardware/mpu6050.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace digirunner_hardware
{

using Trigger = std_srvs::srv::Trigger;

// "x", "-y", ... -> {axis index, sign}
static std::pair<int, double> parse_axis(const std::string & s)
{
  const bool neg = !s.empty() && s[0] == '-';
  const std::string a = s.substr(neg);
  if (a.size() != 1 || a[0] < 'x' || a[0] > 'z') {
    throw std::invalid_argument("axis must be x, -x, y, -y, z or -z (got '" + s + "')");
  }
  return {a[0] - 'x', neg ? -1.0 : 1.0};
}

class ImuNode : public rclcpp::Node
{
public:
  ImuNode()
  : Node("imu_node")
  {
    // Which printed board axis points forward and which points left; up = forward x left.
    const auto fwd = parse_axis(declare_parameter<std::string>("forward_axis", "x"));
    const auto left = parse_axis(declare_parameter<std::string>("left_axis", "y"));
    if (fwd.first == left.first) {throw std::invalid_argument("forward_axis and left_axis clash");}
    for (int i = 0; i < 3; ++i) {
      map_[0][i] = i == fwd.first ? fwd.second : 0;
      map_[1][i] = i == left.first ? left.second : 0;
    }
    for (int i = 0; i < 3; ++i) {  // cross product
      map_[2][i] = map_[0][(i + 1) % 3] * map_[1][(i + 2) % 3] -
        map_[0][(i + 2) % 3] * map_[1][(i + 1) % 3];
    }

    const auto device = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
    const int address = declare_parameter<int>("address", 0x68);
    const double rate_hz = declare_parameter<double>("rate_hz", 100.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    alpha_ = declare_parameter<double>("filter_alpha", 0.98);
    cal_samples_ = declare_parameter<int>("calibration_samples", 200);
    cal_max_std_ = declare_parameter<double>("calibration_max_stddev", 0.02);  // rad/s
    if (rate_hz <= 0 || cal_samples_ < 2) {
      throw std::invalid_argument("need rate_hz > 0 and calibration_samples >= 2");
    }

    imu_ = std::make_unique<Mpu6050>(device, address);
    if (const int id = imu_->who_am_i(); id != 0x68) {
      RCLCPP_WARN(get_logger(), "WHO_AM_I is 0x%02x, not 0x68 (common on clones); continuing", id);
    }
    imu_->configure(
      declare_parameter<int>("dlpf_cfg", 3), declare_parameter<int>("sample_rate_div", 4),
      declare_parameter<int>("accel_range_g", 4), declare_parameter<int>("gyro_range_dps", 500));
    RCLCPP_INFO(get_logger(), "MPU-6050 ready on %s at 0x%02x", device.c_str(), address);

    pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SensorDataQoS().keep_last(1));
    srv_ = create_service<Trigger>(
      "imu/calibrate",
      [this](std::shared_ptr<rclcpp::Service<Trigger>>, std::shared_ptr<rmw_request_id_t> h,
      std::shared_ptr<Trigger::Request>) {waiting_.push_back(*h); start_calibration();});
    if (declare_parameter<bool>("calibrate_on_start", true)) {
      RCLCPP_INFO(get_logger(), "Calibrating gyro bias: keep the robot still...");
      start_calibration();
    }
    period_ = 1.0 / rate_hz;
    timer_ = create_wall_timer(std::chrono::duration<double>(period_), [this] {tick();});
  }

private:
  Vec3 to_robot(const Vec3 & v) const
  {
    Vec3 r{};
    for (int i = 0; i < 3; ++i) {r[i] = map_[i][0] * v[0] + map_[i][1] * v[1] + map_[i][2] * v[2];}
    return r;
  }

  void start_calibration()
  {
    if (cal_n_ < 0) {cal_n_ = 0; cal_sum_ = cal_sq_ = {0, 0, 0};}
  }

  void calibrate_step(const Vec3 & gyro)
  {
    for (int i = 0; i < 3; ++i) {cal_sum_[i] += gyro[i]; cal_sq_[i] += gyro[i] * gyro[i];}
    if (++cal_n_ < cal_samples_) {return;}

    Vec3 mean;
    double spread = 0;
    for (int i = 0; i < 3; ++i) {
      mean[i] = cal_sum_[i] / cal_n_;
      spread = std::max(spread, std::sqrt(std::max(0.0, cal_sq_[i] / cal_n_ - mean[i] * mean[i])));
    }
    cal_n_ = -1;

    Trigger::Response res;
    res.success = spread <= cal_max_std_;
    if (res.success) {
      bias_ = mean;
      res.message = "gyro bias [rad/s]: " + std::to_string(bias_[0]) + ", " +
        std::to_string(bias_[1]) + ", " + std::to_string(bias_[2]);
      RCLCPP_INFO(get_logger(), "Calibrated, %s", res.message.c_str());
    } else {
      res.message = "robot moved (gyro spread " + std::to_string(spread) +
        " rad/s); kept the old bias. Hold it still and retry.";
      RCLCPP_WARN(get_logger(), "%s", res.message.c_str());
    }
    for (auto & h : waiting_) {srv_->send_response(h, res);}
    waiting_.clear();
  }

  void tick()
  {
    // Real dt from the steady clock, so a late tick doesn't skew the gyro integration.
    const auto t = std::chrono::steady_clock::now();
    const double dt =
      last_ ? std::clamp(std::chrono::duration<double>(t - *last_).count(), 0.0, 0.1) : period_;
    last_ = t;

    Mpu6050::Sample s;
    try {
      s = imu_->read();
      if (failures_) {RCLCPP_INFO(get_logger(), "IMU recovered after %d failed reads", failures_);}
      failures_ = 0;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "IMU read failed (%d in a row): %s", ++failures_,
        e.what());
      return;  // publish nothing rather than a stale reading
    }

    const Vec3 a = to_robot(s.accel), raw = to_robot(s.gyro);
    if (cal_n_ >= 0) {calibrate_step(raw);}
    const Vec3 w{raw[0] - bias_[0], raw[1] - bias_[1], raw[2] - bias_[2]};

    // Complementary filter. Body rates -> Euler rates (ZYX), blended with the accel tilt;
    // during foot impacts (|a| far from 1 g) trust the gyro alone.
    const double acc_roll = std::atan2(a[1], a[2]);
    const double acc_pitch = std::atan2(-a[0], std::hypot(a[1], a[2]));
    if (!started_) {
      roll_ = acc_roll, pitch_ = acc_pitch, started_ = true;
    } else {
      const double sr = std::sin(roll_), cr = std::cos(roll_), cp = std::cos(pitch_);
      const double tp = std::abs(cp) > 1e-3 ? std::sin(pitch_) / cp : 0.0;
      const double g = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) / kGravity;
      const double k = (g > 0.5 && g < 1.5) ? alpha_ : 1.0;
      roll_ = k * (roll_ + (w[0] + (sr * w[1] + cr * w[2]) * tp) * dt) + (1 - k) * acc_roll;
      pitch_ = k * (pitch_ + (cr * w[1] - sr * w[2]) * dt) + (1 - k) * acc_pitch;
    }

    sensor_msgs::msg::Imu m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;
    const double cr = std::cos(roll_ / 2), sr = std::sin(roll_ / 2);
    const double cp = std::cos(pitch_ / 2), sp = std::sin(pitch_ / 2);
    m.orientation.x = sr * cp, m.orientation.y = cr * sp;  // RPY -> quaternion, yaw = 0
    m.orientation.z = -sr * sp, m.orientation.w = cr * cp;
    m.angular_velocity.x = w[0], m.angular_velocity.y = w[1], m.angular_velocity.z = w[2];
    m.linear_acceleration.x = a[0], m.linear_acceleration.y = a[1], m.linear_acceleration.z = a[2];
    // Rough variances; yaw is unknown, so its variance is huge.
    m.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 1e6};
    m.angular_velocity_covariance = {1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4};
    m.linear_acceleration_covariance = {1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 1e-2};
    pub_->publish(m);
  }

  double map_[3][3]{};  // rows: robot forward, left, up, in board axes
  std::unique_ptr<Mpu6050> imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Service<Trigger>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
  double period_ = 0.01, alpha_ = 0.98, roll_ = 0, pitch_ = 0;
  bool started_ = false;
  std::optional<std::chrono::steady_clock::time_point> last_;
  int failures_ = 0;
  Vec3 bias_{0, 0, 0}, cal_sum_{}, cal_sq_{};
  int cal_n_ = -1, cal_samples_ = 200;  // cal_n_ < 0: not calibrating
  double cal_max_std_ = 0.02;
  std::vector<rmw_request_id_t> waiting_;
};

}  // namespace digirunner_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<digirunner_hardware::ImuNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("imu_node"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
