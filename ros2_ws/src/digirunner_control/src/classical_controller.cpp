// Copyright 2026 Digi-Runner team. MIT License.
//
// classical_controller: the hand-designed controller that fills the
// controller slot. imu/data + joint_states in, joint_commands out (radians,
// in joint_names order), one 50 Hz timer, no blocking anywhere.
//
//   On start it waits for joint_states, then eases from wherever the joints
//   are to the stand pose. The /walk action (digirunner_interfaces/Walk) runs
//   the open-loop gait for N steps with optional IMU balance; cancelling or
//   finishing a walk eases back to the stand pose.
//
//   Fall detection: if pitch or roll stays past fall_angle_deg for
//   fall_time_ms, the active /walk goal is aborted, servos/set_torque(false)
//   is called and joint_commands stop. Nothing re-enables automatically: a
//   person turns torque back on, then calls ~/reset to stand up again.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "digirunner_control/gait.hpp"
#include "digirunner_interfaces/action/walk.hpp"

namespace digirunner_control
{

class ClassicalController : public rclcpp::Node
{
public:
  using Walk = digirunner_interfaces::action::Walk;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Walk>;

  ClassicalController()
  : Node("classical_controller"),
    fall_detector_(
      declare_parameter<double>("fall_angle_deg", 30.0) * kPi / 180.0,
      declare_parameter<double>("fall_time_ms", 100.0) / 1000.0),
    gait_(load_gait_params(), load_stand_pose())
  {
    const auto rate_hz = declare_parameter<double>("rate_hz", 50.0);
    stand_up_time_ = declare_parameter<double>("stand_up_time", 2.0);
    return_to_stand_time_ = declare_parameter<double>("return_to_stand_time", 1.0);
    imu_timeout_ = declare_parameter<double>("imu_timeout", 0.1);
    require_imu_ = declare_parameter<bool>("require_imu", true);

    balance_enabled_ = declare_parameter<bool>("balance_enabled", false);
    balance_.target_pitch = declare_parameter<double>("balance_target_pitch", 0.0);
    balance_.pitch_gain = declare_parameter<double>("balance_pitch_gain", 0.0);
    balance_.pitch_damping = declare_parameter<double>("balance_pitch_damping", 0.0);
    balance_.roll_gain = declare_parameter<double>("balance_roll_gain", 0.0);
    balance_.roll_damping = declare_parameter<double>("balance_roll_damping", 0.0);
    balance_.max_correction = declare_parameter<double>("balance_max_correction", 0.14);

    if (rate_hz <= 0.0) {
      throw std::invalid_argument("rate_hz must be > 0");
    }

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_imu(*msg);});
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_joint_states(*msg);});

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "joint_commands", rclcpp::QoS(1).reliable());

    torque_client_ = create_client<std_srvs::srv::SetBool>("servos/set_torque");

    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        on_reset(*response);
      });

    walk_server_ = rclcpp_action::create_server<Walk>(
      this, "walk",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Walk::Goal> goal) {
        return on_goal(*goal);
      },
      [](const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        on_goal_accepted(goal_handle);
      });

    last_tick_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz)),
      [this]() {tick();});

    RCLCPP_INFO(get_logger(), "Waiting for joint_states before standing up...");
  }

private:
  enum class State {kWaitingForJointStates, kStandingUp, kStanding, kWalking, kReturning, kFallen};

  // ---------------------------------------------------------------- params

  GaitParams load_gait_params()
  {
    GaitParams p;
    p.cycle_time = declare_parameter<double>("cycle_time", p.cycle_time);
    p.hip_pitch_swing = declare_parameter<double>("hip_pitch_swing", p.hip_pitch_swing);
    p.knee_lift = declare_parameter<double>("knee_lift", p.knee_lift);
    p.hip_roll_sway = declare_parameter<double>("hip_roll_sway", p.hip_roll_sway);
    p.double_support = declare_parameter<double>("double_support", p.double_support);
    p.ramp_cycles = declare_parameter<double>("ramp_cycles", p.ramp_cycles);
    return p;
  }

  JointArray load_stand_pose()
  {
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"left_hip_roll", "left_hip_pitch", "left_knee",
        "right_hip_roll", "right_hip_pitch", "right_knee"});
    const auto pose = declare_parameter<std::vector<double>>(
      "stand_pose", std::vector<double>(kNumJoints, 0.0));
    if (joint_names_.size() != kNumJoints || pose.size() != kNumJoints) {
      throw std::invalid_argument("joint_names and stand_pose must each have 6 entries");
    }
    std::copy(pose.begin(), pose.end(), stand_.begin());
    return stand_;
  }

  // ------------------------------------------------------------ callbacks
  // Subscription callbacks only store the latest message; tick() does the work.

  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    roll_pitch_from_quaternion(
      msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
      roll_, pitch_);
    roll_rate_ = msg.angular_velocity.x;
    pitch_rate_ = msg.angular_velocity.y;
    last_imu_time_ = now();
    have_imu_ = true;
  }

  void on_joint_states(const sensor_msgs::msg::JointState & msg)
  {
    JointArray positions{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), joint_names_[i]);
      const auto idx = static_cast<std::size_t>(std::distance(msg.name.begin(), it));
      if (it == msg.name.end() || idx >= msg.position.size()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "joint_states has no position for '%s'",
          joint_names_[i].c_str());
        return;
      }
      positions[i] = msg.position[idx];
    }
    joint_positions_ = positions;
    have_joint_states_ = true;
  }

  rclcpp_action::GoalResponse on_goal(const Walk::Goal & goal)
  {
    const char * reason = nullptr;
    if (goal.steps <= 0) {
      reason = "steps must be > 0";
    } else if (state_ != State::kStanding) {
      reason = "the robot isn't standing still yet (or has fallen)";
    } else if (require_imu_ && !imu_fresh()) {
      reason = "no recent imu/data (set require_imu:=false to walk without it)";
    }
    if (reason) {
      RCLCPP_WARN(get_logger(), "Rejected /walk goal of %d steps: %s", goal.steps, reason);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void on_goal_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    active_goal_ = goal_handle;
    gait_.start(goal_handle->get_goal()->steps);
    last_feedback_step_ = 0;
    state_ = State::kWalking;
    RCLCPP_INFO(get_logger(), "Walking %d steps", goal_handle->get_goal()->steps);
  }

  void on_reset(std_srvs::srv::Trigger::Response & response)
  {
    if (state_ != State::kFallen) {
      response.success = false;
      response.message = "not fallen; nothing to reset";
      return;
    }
    if (imu_fresh() && fall_detector_.tilted(pitch_, roll_)) {
      response.success = false;
      response.message = "still tilted past the fall threshold; set the robot upright first";
      return;
    }
    fall_detector_.reset();
    have_joint_states_ = false;  // stand up from fresh positions, not pre-fall ones
    state_ = State::kWaitingForJointStates;
    response.success = true;
    response.message = "standing up from the current pose (servo torque must be on)";
    RCLCPP_INFO(get_logger(), "Reset after fall: %s", response.message.c_str());
  }

  // ------------------------------------------------------------ main loop

  void tick()
  {
    const rclcpp::Time t = now();
    const double dt = std::clamp((t - last_tick_).seconds(), 0.0, 0.1);
    last_tick_ = t;

    if (state_ == State::kFallen) {
      return;  // torque is off: send nothing until a person calls ~/reset
    }

    const bool imu_ok = imu_fresh();
    if (imu_ok && fall_detector_.update(pitch_, roll_, dt)) {
      on_fall();
      return;
    }

    if (state_ == State::kWalking && require_imu_ && !imu_ok) {
      RCLCPP_ERROR(get_logger(), "imu/data went silent mid-walk; stopping and standing");
      finish_walk(false);
    }

    switch (state_) {
      case State::kWaitingForJointStates:
        if (!have_joint_states_) {
          return;  // nothing to start from yet; publish nothing
        }
        interpolator_.start(joint_positions_, stand_, stand_up_time_);
        state_ = State::kStandingUp;
        RCLCPP_INFO(get_logger(), "Standing up over %.1f s", stand_up_time_);
        [[fallthrough]];
      case State::kStandingUp:
        command_ = interpolator_.step(dt);
        if (interpolator_.done()) {
          state_ = State::kStanding;
          RCLCPP_INFO(get_logger(), "Standing; ready for /walk goals");
        }
        break;
      case State::kStanding:
        command_ = stand_;
        break;
      case State::kWalking:
        if (active_goal_->is_canceling()) {
          RCLCPP_INFO(get_logger(), "Walk cancelled; returning to stand");
          finish_walk(false);
          command_ = interpolator_.step(dt);
          break;
        }
        command_ = gait_.step(dt);
        if (gait_.current_step() != last_feedback_step_) {
          last_feedback_step_ = gait_.current_step();
          auto feedback = std::make_shared<Walk::Feedback>();
          feedback->current_step = last_feedback_step_;
          active_goal_->publish_feedback(feedback);
        }
        if (gait_.finished()) {
          finish_walk(true);
        }
        break;
      case State::kReturning:
        command_ = interpolator_.step(dt);
        if (interpolator_.done()) {
          state_ = State::kStanding;
        }
        break;
      case State::kFallen:
        return;
    }

    publish(command_, imu_ok);
  }

  // Ends the active goal (succeeded, cancelled or aborted) and eases back to stand.
  void finish_walk(bool success)
  {
    auto result = std::make_shared<Walk::Result>();
    result->success = success;
    result->steps_taken = gait_.steps_taken();
    if (success) {
      active_goal_->succeed(result);
      RCLCPP_INFO(get_logger(), "Walk done: %d steps", result->steps_taken);
    } else if (active_goal_->is_canceling()) {
      active_goal_->canceled(result);
    } else {
      active_goal_->abort(result);
    }
    active_goal_.reset();
    interpolator_.start(command_, stand_, return_to_stand_time_);
    state_ = State::kReturning;
  }

  void on_fall()
  {
    RCLCPP_ERROR(
      get_logger(), "FALL detected: pitch %.1f deg, roll %.1f deg. Aborting walk and "
      "cutting servo torque.", pitch_ * 180.0 / kPi, roll_ * 180.0 / kPi);

    if (active_goal_) {
      auto result = std::make_shared<Walk::Result>();
      result->success = false;
      result->steps_taken = gait_.steps_taken();
      active_goal_->abort(result);
      active_goal_.reset();
    }
    state_ = State::kFallen;

    if (!torque_client_->service_is_ready()) {
      RCLCPP_ERROR(
        get_logger(), "servos/set_torque is not available; switch off servo power by hand!");
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    torque_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        const auto response = future.get();
        if (response->success) {
          RCLCPP_WARN(
            get_logger(), "Servo torque is OFF. Turn it back on, then call ~/reset to stand.");
        } else {
          RCLCPP_ERROR(
            get_logger(), "servos/set_torque(false) failed: %s. Switch off servo power by hand!",
            response->message.c_str());
        }
      });
  }

  // ------------------------------------------------------------ helpers

  bool imu_fresh() const
  {
    return have_imu_ && (now() - last_imu_time_).seconds() < imu_timeout_;
  }

  void publish(const JointArray & base, bool imu_ok)
  {
    JointArray out = base;
    const bool balancing = balance_enabled_ && imu_ok &&
      (state_ == State::kStanding || state_ == State::kWalking || state_ == State::kReturning);
    if (balancing) {
      const auto c = balance_correction(balance_, pitch_, roll_, pitch_rate_, roll_rate_);
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        out[i] += c[i];
      }
    }
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(out.begin(), out.end());
    command_pub_->publish(msg);
  }

  // Parameters
  std::vector<std::string> joint_names_;
  JointArray stand_{};
  double stand_up_time_ = 2.0;
  double return_to_stand_time_ = 1.0;
  double imu_timeout_ = 0.1;
  bool require_imu_ = true;
  bool balance_enabled_ = false;
  BalanceParams balance_;

  // Logic
  FallDetector fall_detector_;
  Gait gait_;
  PoseInterpolator interpolator_;
  State state_ = State::kWaitingForJointStates;
  JointArray command_{};
  int last_feedback_step_ = 0;
  std::shared_ptr<GoalHandle> active_goal_;

  // Latest sensor data
  bool have_imu_ = false;
  rclcpp::Time last_imu_time_;
  double roll_ = 0.0, pitch_ = 0.0, roll_rate_ = 0.0, pitch_rate_ = 0.0;
  bool have_joint_states_ = false;
  JointArray joint_positions_{};
  rclcpp::Time last_tick_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr torque_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp_action::Server<Walk>::SharedPtr walk_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace digirunner_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<digirunner_control::ClassicalController>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("classical_controller"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
