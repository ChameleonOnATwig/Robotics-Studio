// Copyright 2026 Digi-Runner team. MIT License.
//
// Minimal MPU-6050 (GY-521) driver over Linux I2C (/dev/i2c-N). No ROS inside.
// Every failure throws std::runtime_error with a message that says what to check.

#ifndef DIGIRUNNER_HARDWARE__MPU6050_HPP_
#define DIGIRUNNER_HARDWARE__MPU6050_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "digirunner_hardware/imu_math.hpp"

namespace digirunner_hardware
{

class Mpu6050
{
public:
  struct Config
  {
    int dlpf_cfg = 3;          // digital low-pass filter, 0-6 (3 = ~44 Hz)
    int accel_range_g = 4;     // 2, 4, 8 or 16
    int gyro_range_dps = 500;  // 250, 500, 1000 or 2000
    int sample_rate_div = 4;   // internal rate = 1 kHz / (1 + div) when DLPF is on
  };

  // One reading in the board's own axes, SI units.
  struct Sample
  {
    Vec3 accel;          // m/s^2
    Vec3 gyro;           // rad/s
    double temperature;  // degrees C
  };

  Mpu6050(const std::string & device, int address);
  ~Mpu6050();
  Mpu6050(const Mpu6050 &) = delete;
  Mpu6050 & operator=(const Mpu6050 &) = delete;

  // Raw WHO_AM_I register. A genuine MPU-6050 returns 0x68; clones often differ.
  std::uint8_t who_am_i();

  // Reset, wake up with the gyro PLL clock, and set filter and ranges.
  void configure(const Config & config);

  Sample read();

private:
  void write_register(std::uint8_t reg, std::uint8_t value);
  void read_registers(std::uint8_t reg, std::uint8_t * buffer, std::size_t length);

  std::string device_;
  int address_;
  int fd_ = -1;
  double accel_lsb_per_g_ = 8192.0;
  double gyro_lsb_per_dps_ = 65.5;
};

}  // namespace digirunner_hardware

#endif  // DIGIRUNNER_HARDWARE__MPU6050_HPP_
