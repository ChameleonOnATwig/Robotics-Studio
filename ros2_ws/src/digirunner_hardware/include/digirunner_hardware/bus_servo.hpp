// Copyright 2026 Digi-Runner team. MIT License.
// Hiwonder/LewanSoul bus-servo protocol (LX-16A, HTD-35H) over the BusLinker's serial port. No ROS.
// Packet: 55 55 id len cmd params... chk, where len = params + 3 and chk = ~(id+len+cmd+params).
// Replies are parsed packet by packet, so a TX echo on the half-duplex line is skipped safely.

#ifndef DIGIRUNNER_HARDWARE__BUS_SERVO_HPP_
#define DIGIRUNNER_HARDWARE__BUS_SERVO_HPP_

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace digirunner_hardware
{

class BusServo
{
public:
  BusServo(const std::string & port, int baud, int timeout_ms)
  : timeout_ms_(timeout_ms)
  {
    if (baud != 115200) {throw std::invalid_argument("the BusLinker runs at 115200 baud");}
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
    termios t{};
    if (fd_ < 0 || ::tcgetattr(fd_, &t) < 0) {
      const std::string msg = "can't open " + port + " (" + std::strerror(errno) +
        "). Is the BusLinker plugged in and this user in the dialout group?";
      if (fd_ >= 0) {::close(fd_);}
      throw std::runtime_error(msg);
    }
    ::cfmakeraw(&t);
    ::cfsetspeed(&t, B115200);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    ::tcsetattr(fd_, TCSANOW, &t);
    ::tcflush(fd_, TCIOFLUSH);
  }
  ~BusServo() {if (fd_ >= 0) {::close(fd_);}}
  BusServo(const BusServo &) = delete;
  BusServo & operator=(const BusServo &) = delete;

  // SERVO_MOVE_TIME_WRITE: go to pos (0-1000 = 0-240 deg) over time_ms. Also turns torque on.
  void move(int id, int pos, int time_ms)
  {
    send(id, 1, {lo(pos), hi(pos), lo(time_ms), hi(time_ms)});
  }
  // SERVO_LOAD_OR_UNLOAD_WRITE: torque on (hold) or off (limp).
  void set_torque(int id, bool on) {send(id, 31, {static_cast<std::uint8_t>(on)});}
  // SERVO_POS_READ: raw position, or nothing if the servo didn't answer in time.
  std::optional<int> position(int id)
  {
    const auto p = query(id, 28, 2);
    if (!p) {return std::nullopt;}
    return static_cast<std::int16_t>((*p)[0] | (*p)[1] << 8);
  }

  // Send a read command and wait for its reply (id, cmd and length must match).
  std::optional<std::vector<std::uint8_t>> query(int id, std::uint8_t cmd, std::size_t n)
  {
    ::tcflush(fd_, TCIFLUSH);  // drop stale bytes, e.g. echoes of earlier writes
    send(id, cmd, {});
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);
    std::vector<std::uint8_t> b;
    for (;;) {
      while (b.size() >= 2 && (b[0] != 0x55 || b[1] != 0x55)) {b.erase(b.begin());}
      if (b.size() >= 4 && (b[3] < 3 || b[3] > 10)) {b.erase(b.begin()); continue;}  // bad len
      if (b.size() >= 4 && b.size() >= b[3] + 3u) {
        const std::size_t total = b[3] + 3u;
        std::uint8_t sum = 0;
        for (std::size_t i = 2; i + 1 < total; ++i) {sum += b[i];}
        if (static_cast<std::uint8_t>(~sum) != b[total - 1]) {b.erase(b.begin()); continue;}
        if (b[2] == id && b[4] == cmd && b[3] == n + 3) {
          return std::vector<std::uint8_t>(b.begin() + 5, b.begin() + 5 + n);
        }
        b.erase(b.begin(), b.begin() + total);  // a valid packet that isn't our reply (echo)
        continue;
      }
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();
      pollfd pfd{fd_, POLLIN, 0};
      if (left <= 0 || ::poll(&pfd, 1, static_cast<int>(left)) <= 0) {return std::nullopt;}
      std::uint8_t tmp[64];
      const ssize_t r = ::read(fd_, tmp, sizeof(tmp));
      if (r > 0) {b.insert(b.end(), tmp, tmp + r);}
    }
  }

private:
  static std::uint8_t lo(int v) {return static_cast<std::uint8_t>(v & 0xFF);}
  static std::uint8_t hi(int v) {return static_cast<std::uint8_t>((v >> 8) & 0xFF);}

  void send(int id, std::uint8_t cmd, std::initializer_list<std::uint8_t> params)
  {
    std::vector<std::uint8_t> p{0x55, 0x55, static_cast<std::uint8_t>(id),
      static_cast<std::uint8_t>(params.size() + 3), cmd};
    p.insert(p.end(), params);
    std::uint8_t sum = 0;
    for (std::size_t i = 2; i < p.size(); ++i) {sum += p[i];}
    p.push_back(static_cast<std::uint8_t>(~sum));
    if (::write(fd_, p.data(), p.size()) != static_cast<ssize_t>(p.size())) {
      throw std::runtime_error(std::string("serial write failed: ") + std::strerror(errno));
    }
    ::tcdrain(fd_);  // wait until it's on the wire, so reads and writes never overlap
  }

  int fd_ = -1;
  int timeout_ms_;
};

}  // namespace digirunner_hardware

#endif  // DIGIRUNNER_HARDWARE__BUS_SERVO_HPP_
