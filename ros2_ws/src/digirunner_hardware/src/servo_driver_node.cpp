// Copyright 2026 Digi-Runner team. MIT License.
//
// servo_driver_node: the single owner of the BusLinker serial bus, for all six servos (#7).
//   joint_commands (std_msgs/Float64MultiArray, rad, joint order from servos.yaml) -> servos
//   joint_states   (sensor_msgs/JointState, rad) <- servo positions
//   servos/set_torque (std_srvs/SetBool): torque on (hold) / off (limp) on all six
// Raw units <-> radians happens here and nowhere else: rad = direction * (raw - zero) * 0.24 deg.
// Every command is clamped to [min, max]. Fixed bus schedule per tick: write all six, then read
// `reads_per_tick` positions round-robin, so writes and reads never collide.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "digirunner_hardware/bus_servo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace digirunner_hardware
{

using Ints = std::vector<int64_t>;
constexpr double kRadPerUnit = 0.24 * M_PI / 180.0;  // 1000 units = 240 deg

class ServoDriverNode : public rclcpp::Node
{
public:
  ServoDriverNode()
  : Node("servo_driver_node")
  {
    names_ = declare_parameter<std::vector<std::string>>("joints", {});
    ids_ = declare_parameter<Ints>("ids", Ints{});
    zero_ = declare_parameter<Ints>("zero", Ints{});
    dir_ = declare_parameter<Ints>("direction", Ints{});
    min_ = declare_parameter<Ints>("min", Ints{});
    max_ = declare_parameter<Ints>("max", Ints{});
    const auto n = ids_.size();
    if (n == 0 || names_.size() != n || zero_.size() != n || dir_.size() != n ||
      min_.size() != n || max_.size() != n)
    {
      throw std::invalid_argument(
              "joints, ids, zero, direction, min and max must be non-empty and the same length "
              "(load config/servos.yaml with --params-file)");
    }
    for (std::size_t i = 0; i < n; ++i) {
      if (std::abs(dir_[i]) != 1 || min_[i] > zero_[i] || zero_[i] > max_[i] || min_[i] < 0 ||
        max_[i] > 1000)
      {
        throw std::invalid_argument(
                names_[i] + ": need direction +/-1 and 0 <= min <= zero <= max <= 1000");
      }
    }
    const double rate_hz = declare_parameter<double>("rate_hz", 50.0);
    reads_per_tick_ = declare_parameter<int>("reads_per_tick", 2);
    if (rate_hz <= 0 || reads_per_tick_ < 0) {throw std::invalid_argument("bad rate_hz/reads");}
    move_ms_ = static_cast<int>(std::lround(1000.0 / rate_hz));

    bus_ = std::make_unique<BusServo>(
      declare_parameter<std::string>("port", "/dev/ttyUSB0"),
      declare_parameter<int>("baud", 115200), declare_parameter<int>("read_timeout_ms", 5));
    pos_.assign(n, std::nan(""));
    for (std::size_t i = 0; i < n; ++i) {
      if (!read(i)) {
        RCLCPP_WARN(
          get_logger(), "servo %ld (%s) not responding", static_cast<long>(ids_[i]),
          names_[i].c_str());
      }
    }

    pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "joint_commands", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Float64MultiArray & m) {on_command(m.data);});
    srv_ = create_service<std_srvs::srv::SetBool>(
      "servos/set_torque",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        for (auto id : ids_) {bus_->set_torque(id, req->data);}
        torque_ = req->data;
        cmd_.reset();  // never jump to an old target after torque comes back on
        res->success = true;
        res->message = req->data ? "torque on" : "torque off";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(move_ms_), [this] {tick();});
    RCLCPP_INFO(get_logger(), "%zu servos at %.0f Hz", n, rate_hz);
  }

private:
  void on_command(const std::vector<double> & rad)
  {
    if (rad.size() != ids_.size() || !std::all_of(rad.begin(), rad.end(), [](double v) {
        return std::isfinite(v);
      }))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "ignored joint_commands: need %zu finite values",
        ids_.size());
      return;
    }
    if (!torque_) {return;}  // a move would turn torque back on
    std::vector<int> raw(rad.size());
    for (std::size_t i = 0; i < rad.size(); ++i) {
      const long r = zero_[i] + std::lround(dir_[i] * rad[i] / kRadPerUnit);
      raw[i] = static_cast<int>(std::clamp<long>(r, min_[i], max_[i]));
      if (raw[i] != r) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "%s clamped to its limit", names_[i].c_str());
      }
    }
    cmd_ = raw;
  }

  bool read(std::size_t i)
  {
    const auto p = bus_->position(ids_[i]);
    if (p) {pos_[i] = dir_[i] * (*p - zero_[i]) * kRadPerUnit;}
    return p.has_value();
  }

  void tick()
  {
    if (cmd_ && torque_) {
      for (std::size_t i = 0; i < ids_.size(); ++i) {bus_->move(ids_[i], (*cmd_)[i], move_ms_);}
    }
    for (int k = 0; k < reads_per_tick_; ++k) {
      const std::size_t i = next_read_++ % ids_.size();
      if (!read(i)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "no reply from servo %ld (%s)",
          static_cast<long>(ids_[i]), names_[i].c_str());
      }
    }
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = names_;
    js.position = pos_;
    pub_->publish(js);
  }

  std::vector<std::string> names_;
  Ints ids_, zero_, dir_, min_, max_;
  std::unique_ptr<BusServo> bus_;
  std::vector<double> pos_;
  std::optional<std::vector<int>> cmd_;
  bool torque_ = true;
  int move_ms_ = 20, reads_per_tick_ = 2;
  std::size_t next_read_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace digirunner_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<digirunner_hardware::ServoDriverNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("servo_driver_node"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
