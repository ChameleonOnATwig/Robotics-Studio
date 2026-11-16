// Copyright 2026 Digi-Runner team. MIT License.
//
// Gait math with no ROS inside: smooth pose interpolation, the open-loop
// stepping gait, IMU balance correction and fall detection. Header-only and
// ROS-free so it can be unit tested on any machine.
//
// Joint order everywhere (matches the servo ID map 1-6):
//   0 left_hip_roll, 1 left_hip_pitch, 2 left_knee,
//   3 right_hip_roll, 4 right_hip_pitch, 5 right_knee
//
// Sign convention (REP 103: roll joints about +x, pitch joints about +y):
//   hip pitch > 0 : leg swings BACKWARD     knee > 0 : knee BENDS
//   hip roll  > 0 : foot moves LEFT relative to the hip (body shifts right)
//   IMU pitch > 0 : leaning FORWARD         IMU roll > 0 : leaning RIGHT

#ifndef DIGIRUNNER_CONTROL__GAIT_HPP_
#define DIGIRUNNER_CONTROL__GAIT_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace digirunner_control
{

constexpr std::size_t kNumJoints = 6;
using JointArray = std::array<double, kNumJoints>;

enum Joint : std::size_t
{
  kLeftHipRoll = 0, kLeftHipPitch, kLeftKnee,
  kRightHipRoll, kRightHipPitch, kRightKnee
};

constexpr double kPi = 3.14159265358979323846;

// Moves smoothly from one pose to another (cosine ease in/out: zero velocity
// at both ends), so the robot never jumps.
class PoseInterpolator
{
public:
  void start(const JointArray & from, const JointArray & to, double duration)
  {
    from_ = from;
    to_ = to;
    duration_ = std::max(duration, 1e-6);
    elapsed_ = 0.0;
  }

  JointArray step(double dt)
  {
    elapsed_ = std::min(elapsed_ + dt, duration_);
    const double s = 0.5 - 0.5 * std::cos(kPi * elapsed_ / duration_);
    JointArray out;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      out[i] = from_[i] + s * (to_[i] - from_[i]);
    }
    return out;
  }

  bool done() const {return elapsed_ >= duration_;}

private:
  JointArray from_{};
  JointArray to_{};
  double duration_ = 1.0;
  double elapsed_ = 0.0;
};

struct GaitParams
{
  double cycle_time = 3.0;        // s for a full cycle (two steps)
  double hip_pitch_swing = 0.26;  // rad, hip forward/back amplitude ("step length")
  double knee_lift = 0.35;        // rad, extra knee bend at mid-swing ("step height")
  double hip_roll_sway = 0.0;     // rad, side-to-side weight shift onto the stance leg
  double double_support = 0.3;    // 0-0.9, share of the cycle both feet stay down
  double ramp_cycles = 2.0;       // cycles to ease in from zero amplitude

  void validate() const
  {
    if (cycle_time <= 0.0) {throw std::invalid_argument("cycle_time must be > 0");}
    if (double_support < 0.0 || double_support > 0.9) {
      throw std::invalid_argument("double_support must be between 0 and 0.9");
    }
    if (ramp_cycles < 0.0) {throw std::invalid_argument("ramp_cycles must be >= 0");}
  }
};

// Open-loop sinusoidal stepping gait, the same shape as the prototype
// code/walk.py. A phase runs 0 -> 2*pi once per cycle; the right leg runs half
// a cycle behind the left. One step = one foot's swing = half a cycle.
//
// Walking starts and ends at a phase where both feet are down and the
// amplitude ramps up from zero, so the first command equals the stand pose.
class Gait
{
public:
  Gait(const GaitParams & params, const JointArray & stand)
  : params_(params), stand_(stand) {params_.validate();}

  void start(int steps)
  {
    steps_ = std::max(steps, 0);
    phase_ = kStartPhase;
  }

  // Advance by dt seconds and return the joint targets.
  JointArray step(double dt)
  {
    phase_ = std::min(phase_ + 2.0 * kPi * dt / params_.cycle_time, end_phase());
    return pose();
  }

  bool finished() const {return phase_ >= end_phase();}

  // 1-based step in progress (equals steps once finished).
  int current_step() const
  {
    const int s = static_cast<int>(std::floor((phase_ - kStartPhase) / kPi)) + 1;
    return std::min(s, steps_);
  }

  int steps_taken() const
  {
    return finished() ? steps_ : current_step() - 1;
  }

  JointArray pose() const
  {
    const double travelled = phase_ - kStartPhase;
    const double amount = params_.ramp_cycles > 0.0 ?
      std::min(1.0, travelled / (2.0 * kPi * params_.ramp_cycles)) : 1.0;

    JointArray out = stand_;
    leg(phase_, amount, kLeftHipPitch, kLeftKnee, out);
    leg(phase_ + kPi, amount, kRightHipPitch, kRightKnee, out);

    // Shift the body over the stance foot: right while the left foot lifts
    // (left phase near 0), left while the right foot lifts.
    const double sway = amount * params_.hip_roll_sway * std::cos(phase_);
    out[kLeftHipRoll] += sway;
    out[kRightHipRoll] += sway;
    return out;
  }

private:
  // Both feet down, hips centred (sin = -1 but amplitude is 0 at the start).
  static constexpr double kStartPhase = -kPi / 2.0;

  double end_phase() const {return kStartPhase + steps_ * kPi;}

  void leg(double p, double amount, std::size_t hip, std::size_t knee, JointArray & out) const
  {
    // Lift only in the middle of the forward swing (cos p above double_support).
    const double lift = std::max(
      0.0, (std::cos(p) - params_.double_support) / (1.0 - params_.double_support));
    // Leg swings forward (hip pitch decreasing) while sin p rises through 0.
    out[hip] -= amount * params_.hip_pitch_swing * std::sin(p);
    out[knee] += amount * params_.knee_lift * lift;
  }

  GaitParams params_;
  JointArray stand_;
  int steps_ = 0;
  double phase_ = kStartPhase;
};

struct BalanceParams
{
  double target_pitch = 0.0;    // rad, body pitch to hold
  double pitch_gain = 0.0;      // rad of hip pitch per rad of pitch error
  double pitch_damping = 0.0;   // rad of hip pitch per rad/s of pitch rate
  double roll_gain = 0.0;       // rad of hip roll per rad of roll
  double roll_damping = 0.0;    // rad of hip roll per rad/s of roll rate
  double max_correction = 0.14;  // rad (~8 deg), cap on each correction
};

// PD correction on the hips that pushes the body back toward upright: a
// forward lean moves the feet forward (negative hip pitch), a lean to the
// right moves the feet right (negative hip roll). Added to both legs.
inline JointArray balance_correction(
  const BalanceParams & b, double pitch, double roll, double pitch_rate, double roll_rate)
{
  const double lim = std::abs(b.max_correction);
  const double hp = std::clamp(
    -(b.pitch_gain * (pitch - b.target_pitch) + b.pitch_damping * pitch_rate), -lim, lim);
  const double hr = std::clamp(-(b.roll_gain * roll + b.roll_damping * roll_rate), -lim, lim);
  JointArray c{};
  c[kLeftHipPitch] = c[kRightHipPitch] = hp;
  c[kLeftHipRoll] = c[kRightHipRoll] = hr;
  return c;
}

// Latches once pitch or roll stays past the threshold for hold_time seconds.
class FallDetector
{
public:
  FallDetector(double threshold, double hold_time)
  : threshold_(threshold), hold_time_(hold_time) {}

  bool update(double pitch, double roll, double dt)
  {
    if (fallen_) {return true;}
    if (tilted(pitch, roll)) {
      over_time_ += dt;
      if (over_time_ >= hold_time_) {fallen_ = true;}
    } else {
      over_time_ = 0.0;
    }
    return fallen_;
  }

  bool tilted(double pitch, double roll) const
  {
    return std::abs(pitch) > threshold_ || std::abs(roll) > threshold_;
  }

  bool fallen() const {return fallen_;}

  void reset()
  {
    fallen_ = false;
    over_time_ = 0.0;
  }

private:
  double threshold_;
  double hold_time_;
  double over_time_ = 0.0;
  bool fallen_ = false;
};

// Roll and pitch (ZYX Euler) from a unit quaternion.
inline void roll_pitch_from_quaternion(
  double x, double y, double z, double w, double & roll, double & pitch)
{
  roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
}

}  // namespace digirunner_control

#endif  // DIGIRUNNER_CONTROL__GAIT_HPP_
