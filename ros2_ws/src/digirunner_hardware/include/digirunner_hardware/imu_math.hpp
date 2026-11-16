// Copyright 2026 Digi-Runner team. MIT License.
//
// IMU math with no ROS inside: board-to-robot axis mapping, a complementary
// filter for pitch and roll, gyro-bias averaging, and RPY -> quaternion.
// Kept header-only and ROS-free so it can be unit tested on any machine.

#ifndef DIGIRUNNER_HARDWARE__IMU_MATH_HPP_
#define DIGIRUNNER_HARDWARE__IMU_MATH_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace digirunner_hardware
{

using Vec3 = std::array<double, 3>;

constexpr double kGravity = 9.80665;  // m/s^2

inline double dot(const Vec3 & a, const Vec3 & b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

// "x", "-x", "y", "-y", "z" or "-z" as a unit vector in the board's frame.
inline Vec3 axis_from_name(const std::string & name)
{
  if (name == "x") {return {1, 0, 0};}
  if (name == "-x") {return {-1, 0, 0};}
  if (name == "y") {return {0, 1, 0};}
  if (name == "-y") {return {0, -1, 0};}
  if (name == "z") {return {0, 0, 1};}
  if (name == "-z") {return {0, 0, -1};}
  throw std::invalid_argument(
          "axis must be one of x, -x, y, -y, z, -z (got '" + name + "')");
}

// Rotates vectors from the board's printed axes into the robot's REP 103
// frame (x forward, y left, z up). Give the board axis that points forward on
// the robot and the one that points left; up follows from the right-hand rule.
class AxisMap
{
public:
  AxisMap(const std::string & forward_axis, const std::string & left_axis)
  : forward_(axis_from_name(forward_axis)),
    left_(axis_from_name(left_axis)),
    up_(cross(forward_, left_))
  {
    if (dot(forward_, left_) != 0.0) {
      throw std::invalid_argument(
              "forward_axis '" + forward_axis + "' and left_axis '" + left_axis +
              "' must be different, perpendicular board axes");
    }
  }

  Vec3 to_robot(const Vec3 & board) const
  {
    return {dot(board, forward_), dot(board, left_), dot(board, up_)};
  }

private:
  Vec3 forward_;
  Vec3 left_;
  Vec3 up_;
};

// Pitch and roll from gyro + accelerometer, in the robot frame (REP 103):
//   roll  > 0 : leaning RIGHT   (rotation about +x)
//   pitch > 0 : leaning FORWARD (nose down, rotation about +y)
// Same signs as the prototype code/imu.py. Yaw is not estimated: with no
// magnetometer it would only drift.
class ComplementaryFilter
{
public:
  explicit ComplementaryFilter(double alpha = 0.98)
  : alpha_(alpha) {}

  // accel: specific force in m/s^2 (reads +g on z when level and still).
  // gyro: body rates in rad/s, bias already removed. dt: seconds since last call.
  void update(const Vec3 & accel, const Vec3 & gyro, double dt)
  {
    const double acc_roll = std::atan2(accel[1], accel[2]);
    const double acc_pitch = std::atan2(-accel[0], std::hypot(accel[1], accel[2]));

    if (!initialized_) {
      roll_ = acc_roll;
      pitch_ = acc_pitch;
      initialized_ = true;
      return;
    }

    // Body rates -> Euler angle rates (ZYX convention).
    const double sr = std::sin(roll_);
    const double cr = std::cos(roll_);
    const double cp = std::cos(pitch_);
    const double tp = std::abs(cp) > 1e-3 ? std::sin(pitch_) / cp : 0.0;
    roll_rate_ = gyro[0] + (sr * gyro[1] + cr * gyro[2]) * tp;
    pitch_rate_ = cr * gyro[1] - sr * gyro[2];

    // During foot impacts the accelerometer sees far more than gravity; trust
    // only the gyro for those samples.
    const double accel_g = std::sqrt(dot(accel, accel)) / kGravity;
    const double a = (accel_g > 0.5 && accel_g < 1.5) ? alpha_ : 1.0;

    roll_ = a * (roll_ + roll_rate_ * dt) + (1.0 - a) * acc_roll;
    pitch_ = a * (pitch_ + pitch_rate_ * dt) + (1.0 - a) * acc_pitch;
  }

  bool initialized() const {return initialized_;}
  double roll() const {return roll_;}
  double pitch() const {return pitch_;}
  double roll_rate() const {return roll_rate_;}
  double pitch_rate() const {return pitch_rate_;}

private:
  double alpha_;
  bool initialized_ = false;
  double roll_ = 0.0;
  double pitch_ = 0.0;
  double roll_rate_ = 0.0;
  double pitch_rate_ = 0.0;
};

// Averages gyro readings while the robot stands still, and checks it really
// was still (low spread) before the result is trusted.
class GyroBiasEstimator
{
public:
  void reset()
  {
    n_ = 0;
    sum_ = {0, 0, 0};
    sum_sq_ = {0, 0, 0};
  }

  void add(const Vec3 & gyro)
  {
    for (std::size_t i = 0; i < 3; ++i) {
      sum_[i] += gyro[i];
      sum_sq_[i] += gyro[i] * gyro[i];
    }
    ++n_;
  }

  std::size_t count() const {return n_;}

  Vec3 mean() const
  {
    if (n_ == 0) {return {0, 0, 0};}
    return {sum_[0] / n_, sum_[1] / n_, sum_[2] / n_};
  }

  // Largest per-axis standard deviation, rad/s.
  double max_stddev() const
  {
    if (n_ < 2) {return 0.0;}
    const Vec3 m = mean();
    double worst = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
      const double var = std::max(0.0, sum_sq_[i] / n_ - m[i] * m[i]);
      worst = std::max(worst, std::sqrt(var));
    }
    return worst;
  }

private:
  std::size_t n_ = 0;
  Vec3 sum_{0, 0, 0};
  Vec3 sum_sq_{0, 0, 0};
};

struct Quaternion
{
  double x;
  double y;
  double z;
  double w;
};

inline Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return {
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy};
}

}  // namespace digirunner_hardware

#endif  // DIGIRUNNER_HARDWARE__IMU_MATH_HPP_
