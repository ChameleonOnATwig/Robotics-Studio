// Copyright 2026 Digi-Runner team. MIT License.
// Minimal MPU-6050 (GY-521) driver over Linux I2C. No ROS inside. Errors throw std::runtime_error.

#ifndef DIGIRUNNER_HARDWARE__MPU6050_HPP_
#define DIGIRUNNER_HARDWARE__MPU6050_HPP_

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

namespace digirunner_hardware
{

using Vec3 = std::array<double, 3>;
constexpr double kGravity = 9.80665;  // m/s^2

class Mpu6050
{
public:
  struct Sample {Vec3 accel, gyro;};  // board axes; m/s^2 and rad/s

  Mpu6050(const std::string & device, int address)
  {
    fd_ = ::open(device.c_str(), O_RDWR);
    if (fd_ < 0 || ::ioctl(fd_, I2C_SLAVE, address) < 0) {
      const std::string msg = "can't open " + device + " (" + std::strerror(errno) +
        "). Is I2C enabled (raspi-config nonint do_i2c 0) and this user in the i2c group?";
      if (fd_ >= 0) {::close(fd_);}
      throw std::runtime_error(msg);
    }
  }
  ~Mpu6050() {::close(fd_);}
  Mpu6050(const Mpu6050 &) = delete;
  Mpu6050 & operator=(const Mpu6050 &) = delete;

  std::uint8_t who_am_i() {std::uint8_t id; read(0x75, &id, 1); return id;}  // 0x68 if genuine

  // Reset, wake on the X-gyro clock, set the low-pass filter (0-6), sample divider and ranges.
  void configure(int dlpf, int rate_div, int accel_g, int gyro_dps)
  {
    const int a = range_bits(accel_g, 2), g = range_bits(gyro_dps, 250);
    if (a < 0 || g < 0 || dlpf < 0 || dlpf > 6 || rate_div < 0 || rate_div > 255) {
      throw std::invalid_argument(
              "bad IMU config: dlpf 0-6, sample_rate_div 0-255, accel_range_g 2/4/8/16, "
              "gyro_range_dps 250/500/1000/2000");
    }
    accel_scale_ = kGravity * accel_g / 32768.0;
    gyro_scale_ = gyro_dps * (3.14159265358979323846 / 180.0) / 32768.0;
    using std::chrono::milliseconds;
    write(0x6B, 0x80); std::this_thread::sleep_for(milliseconds(100));  // PWR_MGMT_1: reset
    write(0x6B, 0x01); std::this_thread::sleep_for(milliseconds(50));   // wake, PLL clock
    write(0x19, rate_div);   // SMPLRT_DIV
    write(0x1A, dlpf);       // CONFIG
    write(0x1B, g << 3);     // GYRO_CONFIG
    write(0x1C, a << 3);     // ACCEL_CONFIG
  }

  Sample read()
  {
    std::uint8_t b[14];  // ACCEL_XOUT_H..GYRO_ZOUT_L: accel xyz, temperature, gyro xyz
    read(0x3B, b, sizeof(b));
    auto v = [&](int i) {return static_cast<std::int16_t>(b[i] << 8 | b[i + 1]);};
    return {{v(0) * accel_scale_, v(2) * accel_scale_, v(4) * accel_scale_},
      {v(8) * gyro_scale_, v(10) * gyro_scale_, v(12) * gyro_scale_}};
  }

private:
  // 0..3 when range is base, 2*base, 4*base or 8*base; else -1.
  static int range_bits(int range, int base)
  {
    for (int i = 0; i < 4; ++i) {if (range == base << i) {return i;}}
    return -1;
  }

  void fail(const char * what, int reg)
  {
    throw std::runtime_error(
            std::string("I2C ") + what + " of register " + std::to_string(reg) + " failed (" +
            std::strerror(errno) + "). Check the wiring: `i2cdetect -y 1` should show 68.");
  }
  void write(std::uint8_t reg, int value)
  {
    const std::uint8_t buf[2] = {reg, static_cast<std::uint8_t>(value)};
    if (::write(fd_, buf, 2) != 2) {fail("write", reg);}
  }
  void read(std::uint8_t reg, std::uint8_t * out, std::size_t n)
  {
    if (::write(fd_, &reg, 1) != 1 || ::read(fd_, out, n) != static_cast<ssize_t>(n)) {
      fail("read", reg);
    }
  }

  int fd_ = -1;
  double accel_scale_ = 0, gyro_scale_ = 0;
};

}  // namespace digirunner_hardware

#endif  // DIGIRUNNER_HARDWARE__MPU6050_HPP_
