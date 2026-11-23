#include "relbot_hardware_cpp/serial_mcu_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace relbot::hw {

uint8_t computeXorChecksum(const std::string& payload) {
  uint8_t cs = 0;
  for (unsigned char c : payload) {
    cs ^= c;
  }
  return cs;
}

std::string encodeCommandFrame(const ManipulatorCommand& cmd) {
  const uint16_t dxl_ticks = radiansToXl430Ticks(cmd.arm_angle_rad);
  const uint16_t dxl_vel = radPerSecToXl430Velocity(cmd.arm_velocity_rad_s);
  const uint16_t mg996r_us = degreesToMg996rPulseUs(cmd.gripper_angle_deg);
  const int torque = cmd.torque_enable ? 1 : 0;

  std::ostringstream payload;
  payload << "CMD," << dxl_ticks << "," << dxl_vel << "," << mg996r_us << "," << torque;
  const std::string payload_str = payload.str();
  const uint8_t cs = computeXorChecksum(payload_str);

  char frame[128];
  std::snprintf(frame, sizeof(frame), "$%s*%02X\n", payload_str.c_str(), cs);
  return std::string(frame);
}

bool decodeTelemetryFrame(const std::string& line,
                          const ManipulatorCommand& last_cmd,
                          double angle_tolerance_rad,
                          ManipulatorTelemetry& out_telemetry) {
  const auto dollar_pos = line.find('$');
  const auto star_pos = line.find('*');
  if (dollar_pos == std::string::npos || star_pos == std::string::npos ||
      star_pos <= dollar_pos + 1 || star_pos + 2 >= line.size()) {
    return false;
  }

  const std::string payload = line.substr(dollar_pos + 1, star_pos - dollar_pos - 1);
  const std::string cs_hex = line.substr(star_pos + 1, 2);

  unsigned int received_cs = 0;
  if (std::sscanf(cs_hex.c_str(), "%02x", &received_cs) != 1) {
    return false;
  }
  if (static_cast<uint8_t>(received_cs) != computeXorChecksum(payload)) {
    return false;
  }

  // Expected payload: STAT,<dxl_ticks>,<dxl_vel_units>,<load_ma>,<mg996r_us>,<grasped>,<err>
  std::vector<std::string> tokens;
  std::stringstream ss(payload);
  std::string item;
  while (std::getline(ss, item, ',')) {
    tokens.push_back(item);
  }
  if (tokens.size() < 7 || tokens[0] != "STAT") {
    return false;
  }

  try {
    const uint16_t dxl_ticks = static_cast<uint16_t>(std::stoi(tokens[1]));
    const int dxl_vel_units = std::stoi(tokens[2]);
    const double load_ma = std::stod(tokens[3]);
    const uint16_t mg996r_us = static_cast<uint16_t>(std::stoi(tokens[4]));
    const bool grasped = (std::stoi(tokens[5]) != 0);
    const uint8_t err = static_cast<uint8_t>(std::stoi(tokens[6]));

    constexpr double kRadPerSecPerUnit = (0.229 * 2.0 * M_PI) / 60.0;
    out_telemetry.current_arm_angle_rad = xl430TicksToRadians(dxl_ticks);
    out_telemetry.current_arm_velocity_rad_s = static_cast<double>(dxl_vel_units) * kRadPerSecPerUnit;
    out_telemetry.arm_load_ma = load_ma;
    out_telemetry.current_gripper_angle_deg = mg996rPulseUsToDegrees(mg996r_us);
    out_telemetry.object_grasped = grasped;
    out_telemetry.error_code = err;
    out_telemetry.target_reached =
        std::abs(out_telemetry.current_arm_angle_rad - last_cmd.arm_angle_rad) <= angle_tolerance_rad;
    out_telemetry.status_message = (err == 0) ? "OK" : "HW_FAULT_" + std::to_string(err);
    return true;
  } catch (...) {
    return false;
  }
}

SerialMcuBridge::SerialMcuBridge(std::string port_name, int baud_rate, bool simulate_if_unavailable)
    : port_name_(std::move(port_name)),
      baud_rate_(baud_rate),
      simulate_if_unavailable_(simulate_if_unavailable) {}

SerialMcuBridge::~SerialMcuBridge() {
  disconnect();
}

bool SerialMcuBridge::connect() {
  disconnect();

  fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    if (simulate_if_unavailable_) {
      simulation_mode_ = true;
      sim_state_.current_arm_angle_rad = 0.10;
      sim_state_.current_gripper_angle_deg = 20.0;
      sim_state_.status_message = "SIMULATION_MODE";
      return true;
    }
    return false;
  }

  struct termios tty {};
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    if (simulate_if_unavailable_) {
      simulation_mode_ = true;
      return true;
    }
    return false;
  }

  speed_t speed = B115200;
  if (baud_rate_ == 57600) speed = B57600;
  else if (baud_rate_ == 230400) speed = B230400;
  else if (baud_rate_ == 1000000) speed = B1000000;

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  simulation_mode_ = false;
  return true;
}

void SerialMcuBridge::disconnect() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  simulation_mode_ = false;
}

bool SerialMcuBridge::sendCommand(const ManipulatorCommand& cmd) {
  last_cmd_ = cmd;
  const std::string frame = encodeCommandFrame(cmd);

  if (simulation_mode_) {
    return true;
  }
  if (fd_ < 0) {
    return false;
  }
  const ssize_t written = ::write(fd_, frame.data(), frame.size());
  return written == static_cast<ssize_t>(frame.size());
}

void SerialMcuBridge::stepSimulation(double dt_sec) {
  const double max_step = std::max(0.1, last_cmd_.arm_velocity_rad_s) * dt_sec;
  const double arm_err = last_cmd_.arm_angle_rad - sim_state_.current_arm_angle_rad;

  if (std::abs(arm_err) <= max_step) {
    sim_state_.current_arm_angle_rad = last_cmd_.arm_angle_rad;
    sim_state_.current_arm_velocity_rad_s = 0.0;
    sim_state_.target_reached = true;
  } else {
    const double dir = (arm_err > 0.0) ? 1.0 : -1.0;
    sim_state_.current_arm_angle_rad += dir * max_step;
    sim_state_.current_arm_velocity_rad_s = dir * last_cmd_.arm_velocity_rad_s;
    sim_state_.target_reached = false;
  }

  // Simulate MG996R servo slew rate (~300 deg/sec)
  const double max_servo_step = 300.0 * dt_sec;
  const double grip_err = last_cmd_.gripper_angle_deg - sim_state_.current_gripper_angle_deg;
  if (std::abs(grip_err) <= max_servo_step) {
    sim_state_.current_gripper_angle_deg = last_cmd_.gripper_angle_deg;
  } else {
    sim_state_.current_gripper_angle_deg += ((grip_err > 0.0) ? 1.0 : -1.0) * max_servo_step;
  }

  sim_state_.object_grasped = (sim_state_.current_gripper_angle_deg >= 85.0);
  sim_state_.arm_load_ma = sim_state_.object_grasped ? 320.0 : 110.0;
  sim_state_.error_code = 0;
  sim_state_.status_message = "SIMULATION_OK";
}

bool SerialMcuBridge::pollTelemetry(ManipulatorTelemetry& telemetry, double dt_sec) {
  if (simulation_mode_) {
    stepSimulation(dt_sec);
    telemetry = sim_state_;
    return true;
  }
  if (fd_ < 0) {
    return false;
  }

  char buf[256];
  const ssize_t n = ::read(fd_, buf, sizeof(buf));
  if (n > 0) {
    rx_buffer_.append(buf, static_cast<size_t>(n));
  }

  bool decoded_any = false;
  std::size_t newline_pos = std::string::npos;
  while ((newline_pos = rx_buffer_.find('\n')) != std::string::npos) {
    std::string line = rx_buffer_.substr(0, newline_pos);
    rx_buffer_.erase(0, newline_pos + 1);
    if (decodeTelemetryFrame(line, last_cmd_, 0.04, telemetry)) {
      decoded_any = true;
    }
  }
  return decoded_any;
}

}  // namespace relbot::hw
