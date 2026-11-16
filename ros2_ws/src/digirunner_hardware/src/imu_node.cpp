// Copyright 2026 Digi-Runner team. MIT License.
//
// imu_node: reads the GY-521 (MPU-6050) over I2C and publishes imu/data
// (sensor_msgs/Imu) at 100 Hz in the robot's REP 103 frame, with pitch and
// roll from a complementary filter. Yaw is always 0 (no magnetometer).
//
// Services:
//   imu/calibrate (std_srvs/Trigger): average the gyro bias while the robot
//   stands still. Replies once the samples are collected (~2 s by default),
//   without blocking the publish loop.

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "digirunner_hardware/imu_math.hpp"
#include "digirunner_hardware/mpu6050.hpp"

namespace digirunner_hardware
{

class ImuNode : public rclcpp::Node
{
public:
  ImuNode()
  : Node("imu_node"),
    axis_map_(
      declare_parameter<std::string>("forward_axis", "x"),
      declare_parameter<std::string>("left_axis", "y")),
    filter_(declare_parameter<double>("filter_alpha", 0.98))
  {
    const auto device = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
    const auto address = declare_parameter<int>("address", 0x68);
    const auto rate_hz = declare_parameter<double>("rate_hz", 100.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    calibration_samples_ = declare_parameter<int>("calibration_samples", 200);
    max_still_stddev_ = declare_parameter<double>("calibration_max_stddev", 0.02);  // rad/s

    Mpu6050::Config config;
    config.dlpf_cfg = declare_parameter<int>("dlpf_cfg", 3);
    config.accel_range_g = declare_parameter<int>("accel_range_g", 4);
    config.gyro_range_dps = declare_parameter<int>("gyro_range_dps", 500);
    config.sample_rate_div = declare_parameter<int>("sample_rate_div", 4);
    const bool calibrate_on_start = declare_parameter<bool>("calibrate_on_start", true);

    if (rate_hz <= 0.0 || calibration_samples_ < 2) {
      throw std::invalid_argument("rate_hz must be > 0 and calibration_samples >= 2");
    }

    imu_ = std::make_unique<Mpu6050>(device, static_cast<int>(address));
    const auto id = imu_->who_am_i();
    if (id != 0x68) {
      RCLCPP_WARN(
        get_logger(), "WHO_AM_I is 0x%02x (a genuine MPU-6050 is 0x68); continuing, "
        "many clone boards differ.", id);
    }
    imu_->configure(config);
    RCLCPP_INFO(
      get_logger(), "MPU-6050 on %s at 0x%02x: DLPF %d, +/-%d g, +/-%d deg/s",
      device.c_str(), static_cast<int>(address), config.dlpf_cfg,
      config.accel_range_g, config.gyro_range_dps);

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SensorDataQoS().keep_last(1));

    calibrate_service_ = create_service<std_srvs::srv::Trigger>(
      "imu/calibrate",
      [this](
        std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>>/*service*/,
        std::shared_ptr<rmw_request_id_t> header,
        std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/) {
        pending_calibration_requests_.push_back(*header);
        start_calibration();
      });

    if (calibrate_on_start) {
      RCLCPP_INFO(get_logger(), "Calibrating gyro bias: keep the robot still...");
      start_calibration();
    }

    period_ = 1.0 / rate_hz;
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_)),
      [this]() {tick();});
  }

private:
  void start_calibration()
  {
    if (!calibrating_) {
      calibrating_ = true;
      bias_estimator_.reset();
    }
  }

  void finish_calibration()
  {
    calibrating_ = false;
    std_srvs::srv::Trigger::Response response;
    const double spread = bias_estimator_.max_stddev();
    if (spread > max_still_stddev_) {
      response.success = false;
      response.message = "robot moved during calibration (gyro spread " +
        std::to_string(spread) + " rad/s); kept the previous bias. Hold it still and retry.";
      RCLCPP_WARN(get_logger(), "%s", response.message.c_str());
    } else {
      bias_ = bias_estimator_.mean();
      response.success = true;
      response.message = "gyro bias [rad/s]: " + std::to_string(bias_[0]) + ", " +
        std::to_string(bias_[1]) + ", " + std::to_string(bias_[2]);
      RCLCPP_INFO(get_logger(), "Calibrated, %s", response.message.c_str());
    }
    for (auto & header : pending_calibration_requests_) {
      calibrate_service_->send_response(header, response);
    }
    pending_calibration_requests_.clear();
  }

  void tick()
  {
    // dt from the steady clock, so a late tick doesn't confuse the gyro integration.
    const auto now_steady = std::chrono::steady_clock::now();
    double dt = period_;
    if (have_last_tick_) {
      dt = std::chrono::duration<double>(now_steady - last_tick_).count();
      dt = std::min(std::max(dt, 0.0), 0.1);
    }
    last_tick_ = now_steady;
    have_last_tick_ = true;

    Mpu6050::Sample sample;
    try {
      sample = imu_->read();
      if (consecutive_failures_ > 0) {
        RCLCPP_INFO(
          get_logger(), "IMU reads recovered after %d failures", consecutive_failures_);
      }
      consecutive_failures_ = 0;
    } catch (const std::exception & e) {
      ++consecutive_failures_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "IMU read failed (%d in a row): %s",
        consecutive_failures_, e.what());
      return;  // publish nothing rather than a stale reading
    }

    const Vec3 accel = axis_map_.to_robot(sample.accel);
    const Vec3 gyro_raw = axis_map_.to_robot(sample.gyro);

    if (calibrating_) {
      bias_estimator_.add(gyro_raw);
      if (bias_estimator_.count() >= static_cast<std::size_t>(calibration_samples_)) {
        finish_calibration();
      }
    }

    const Vec3 gyro{gyro_raw[0] - bias_[0], gyro_raw[1] - bias_[1], gyro_raw[2] - bias_[2]};
    filter_.update(accel, gyro, dt);

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    const auto q = quaternion_from_rpy(filter_.roll(), filter_.pitch(), 0.0);
    msg.orientation.x = q.x;
    msg.orientation.y = q.y;
    msg.orientation.z = q.z;
    msg.orientation.w = q.w;

    msg.angular_velocity.x = gyro[0];
    msg.angular_velocity.y = gyro[1];
    msg.angular_velocity.z = gyro[2];
    msg.linear_acceleration.x = accel[0];
    msg.linear_acceleration.y = accel[1];
    msg.linear_acceleration.z = accel[2];

    // Rough variances. Yaw is unknown, so its variance is huge.
    msg.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 1e6};
    msg.angular_velocity_covariance = {1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4};
    msg.linear_acceleration_covariance = {1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 1e-2};

    publisher_->publish(msg);
  }

  AxisMap axis_map_;
  ComplementaryFilter filter_;
  std::unique_ptr<Mpu6050> imu_;
  std::string frame_id_;
  double period_ = 0.01;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::chrono::steady_clock::time_point last_tick_;
  bool have_last_tick_ = false;
  int consecutive_failures_ = 0;

  Vec3 bias_{0, 0, 0};
  GyroBiasEstimator bias_estimator_;
  bool calibrating_ = false;
  int calibration_samples_ = 200;
  double max_still_stddev_ = 0.02;
  std::vector<rmw_request_id_t> pending_calibration_requests_;
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
