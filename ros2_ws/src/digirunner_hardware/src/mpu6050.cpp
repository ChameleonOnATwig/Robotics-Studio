// Copyright 2026 Digi-Runner team. MIT License.

#include "digirunner_hardware/mpu6050.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace digirunner_hardware
{

namespace
{

// Register map (MPU-6000/6050 Register Map, rev 4.2).
constexpr std::uint8_t kSmplrtDiv = 0x19;
constexpr std::uint8_t kConfig = 0x1A;
constexpr std::uint8_t kGyroConfig = 0x1B;
constexpr std::uint8_t kAccelConfig = 0x1C;
constexpr std::uint8_t kAccelXoutH = 0x3B;  // 14 bytes: accel xyz, temp, gyro xyz
constexpr std::uint8_t kPwrMgmt1 = 0x6B;
constexpr std::uint8_t kWhoAmI = 0x75;

constexpr std::uint8_t kDeviceReset = 0x80;
constexpr std::uint8_t kClockPllXGyro = 0x01;  // wake up, X gyro as clock (more stable)

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

std::int16_t be16(const std::uint8_t * p)
{
  return static_cast<std::int16_t>((static_cast<std::uint16_t>(p[0]) << 8) | p[1]);
}

std::string hex(int value)
{
  std::ostringstream s;
  s << "0x" << std::hex << value;
  return s.str();
}

}  // namespace

Mpu6050::Mpu6050(const std::string & device, int address)
: device_(device), address_(address)
{
  fd_ = ::open(device_.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error(
            "can't open " + device_ + " (" + std::strerror(errno) +
            "). Enable I2C with `sudo raspi-config nonint do_i2c 0`, reboot, and make sure "
            "this user is in the i2c group.");
  }
  if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
    const std::string err = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("can't select I2C address " + hex(address_) + " (" + err + ")");
  }
}

Mpu6050::~Mpu6050()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

std::uint8_t Mpu6050::who_am_i()
{
  std::uint8_t id = 0;
  read_registers(kWhoAmI, &id, 1);
  return id;
}

void Mpu6050::configure(const Config & config)
{
  if (config.dlpf_cfg < 0 || config.dlpf_cfg > 6) {
    throw std::invalid_argument("dlpf_cfg must be 0-6");
  }
  if (config.sample_rate_div < 0 || config.sample_rate_div > 255) {
    throw std::invalid_argument("sample_rate_div must be 0-255");
  }

  std::uint8_t accel_bits;
  switch (config.accel_range_g) {
    case 2: accel_bits = 0; accel_lsb_per_g_ = 16384.0; break;
    case 4: accel_bits = 1; accel_lsb_per_g_ = 8192.0; break;
    case 8: accel_bits = 2; accel_lsb_per_g_ = 4096.0; break;
    case 16: accel_bits = 3; accel_lsb_per_g_ = 2048.0; break;
    default: throw std::invalid_argument("accel_range_g must be 2, 4, 8 or 16");
  }

  std::uint8_t gyro_bits;
  switch (config.gyro_range_dps) {
    case 250: gyro_bits = 0; gyro_lsb_per_dps_ = 131.0; break;
    case 500: gyro_bits = 1; gyro_lsb_per_dps_ = 65.5; break;
    case 1000: gyro_bits = 2; gyro_lsb_per_dps_ = 32.8; break;
    case 2000: gyro_bits = 3; gyro_lsb_per_dps_ = 16.4; break;
    default: throw std::invalid_argument("gyro_range_dps must be 250, 500, 1000 or 2000");
  }

  // Startup only, so short sleeps are fine here (never call this from a callback).
  write_register(kPwrMgmt1, kDeviceReset);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  write_register(kPwrMgmt1, kClockPllXGyro);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  write_register(kSmplrtDiv, static_cast<std::uint8_t>(config.sample_rate_div));
  write_register(kConfig, static_cast<std::uint8_t>(config.dlpf_cfg));
  write_register(kGyroConfig, static_cast<std::uint8_t>(gyro_bits << 3));
  write_register(kAccelConfig, static_cast<std::uint8_t>(accel_bits << 3));
}

Mpu6050::Sample Mpu6050::read()
{
  std::uint8_t b[14];
  read_registers(kAccelXoutH, b, sizeof(b));

  const double g_per_lsb = kGravity / accel_lsb_per_g_;
  const double rad_per_lsb = kDegToRad / gyro_lsb_per_dps_;

  Sample s;
  s.accel = {be16(&b[0]) * g_per_lsb, be16(&b[2]) * g_per_lsb, be16(&b[4]) * g_per_lsb};
  s.temperature = be16(&b[6]) / 340.0 + 36.53;
  s.gyro = {be16(&b[8]) * rad_per_lsb, be16(&b[10]) * rad_per_lsb, be16(&b[12]) * rad_per_lsb};
  return s;
}

void Mpu6050::write_register(std::uint8_t reg, std::uint8_t value)
{
  const std::uint8_t buf[2] = {reg, value};
  if (::write(fd_, buf, 2) != 2) {
    throw std::runtime_error(
            "I2C write to register " + hex(reg) + " failed (" + std::strerror(errno) +
            "). Check the IMU wiring; `i2cdetect -y 1` should show " + hex(address_) + ".");
  }
}

void Mpu6050::read_registers(std::uint8_t reg, std::uint8_t * buffer, std::size_t length)
{
  if (::write(fd_, &reg, 1) != 1) {
    throw std::runtime_error(
            "I2C register select " + hex(reg) + " failed (" + std::strerror(errno) +
            "). Check the IMU wiring; `i2cdetect -y 1` should show " + hex(address_) + ".");
  }
  const ssize_t n = ::read(fd_, buffer, length);
  if (n != static_cast<ssize_t>(length)) {
    throw std::runtime_error(
            "I2C read from register " + hex(reg) + " failed (" +
            (n < 0 ? std::strerror(errno) : "short read") + ")");
  }
}

}  // namespace digirunner_hardware
