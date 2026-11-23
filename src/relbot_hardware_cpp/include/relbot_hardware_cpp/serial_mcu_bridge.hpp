#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace relbot::hw {

/**
 * @brief Convert radians to Dynamixel XL430-W250-T raw position ticks [0..4095].
 *
 * Dynamixel XL430 uses 4096 ticks per 360 degrees (0.087890625 deg/tick).
 * Center (0.0 rad) is mapped to tick 2048 (180 deg).
 */
inline uint16_t radiansToXl430Ticks(double angle_rad) {
  constexpr double kCenterTick = 2048.0;
  constexpr double kTicksPerRadian = 4096.0 / (2.0 * M_PI);
  double raw = std::round(kCenterTick + angle_rad * kTicksPerRadian);
  if (raw < 0.0) raw = 0.0;
  if (raw > 4095.0) raw = 4095.0;
  return static_cast<uint16_t>(raw);
}

/**
 * @brief Convert Dynamixel XL430-W250-T raw position ticks [0..4095] to radians.
 */
inline double xl430TicksToRadians(uint16_t ticks) {
  constexpr double kCenterTick = 2048.0;
  constexpr double kRadiansPerTick = (2.0 * M_PI) / 4096.0;
  return (static_cast<double>(ticks) - kCenterTick) * kRadiansPerTick;
}

/**
 * @brief Convert rad/s angular velocity to Dynamixel XL430 profile velocity units.
 *
 * 1 unit = 0.229 rpm = 0.0239808 rad/s.
 */
inline uint16_t radPerSecToXl430Velocity(double vel_rad_s) {
  if (vel_rad_s <= 0.0) return 0; // 0 means maximum / default profile velocity on XL430
  constexpr double kRadPerSecPerUnit = (0.229 * 2.0 * M_PI) / 60.0;
  double units = std::round(std::abs(vel_rad_s) / kRadPerSecPerUnit);
  if (units < 1.0) units = 1.0;
  if (units > 1023.0) units = 1023.0;
  return static_cast<uint16_t>(units);
}

/**
 * @brief Convert MG996R gripper angle in degrees [0..180] to PWM pulse width in microseconds.
 *
 * Standard MG996R calibration: 0 deg -> 500 us, 180 deg -> 2400 us.
 */
inline uint16_t degreesToMg996rPulseUs(double angle_deg) {
  double clamped = angle_deg;
  if (clamped < 0.0) clamped = 0.0;
  if (clamped > 180.0) clamped = 180.0;
  constexpr double kMinPulseUs = 500.0;
  constexpr double kMaxPulseUs = 2400.0;
  return static_cast<uint16_t>(
      std::round(kMinPulseUs + (clamped / 180.0) * (kMaxPulseUs - kMinPulseUs)));
}

/**
 * @brief Convert MG996R PWM pulse width in microseconds [500..2400] back to degrees.
 */
inline double mg996rPulseUsToDegrees(uint16_t pulse_us) {
  constexpr double kMinPulseUs = 500.0;
  constexpr double kMaxPulseUs = 2400.0;
  double clamped = static_cast<double>(pulse_us);
  if (clamped < kMinPulseUs) clamped = kMinPulseUs;
  if (clamped > kMaxPulseUs) clamped = kMaxPulseUs;
  return ((clamped - kMinPulseUs) / (kMaxPulseUs - kMinPulseUs)) * 180.0;
}

/**
 * @brief Command structure sent from the C++ driver to the Microcontroller bridge.
 */
struct ManipulatorCommand {
  double arm_angle_rad{0.0};
  double arm_velocity_rad_s{1.0};
  double gripper_angle_deg{15.0};
  bool torque_enable{true};
};

/**
 * @brief Telemetry structure decoded from the Microcontroller bridge.
 */
struct ManipulatorTelemetry {
  double current_arm_angle_rad{0.0};
  double current_arm_velocity_rad_s{0.0};
  double arm_load_ma{0.0};
  double current_gripper_angle_deg{15.0};
  bool target_reached{false};
  bool object_grasped{false};
  uint8_t error_code{0};
  std::string status_message{"OK"};
};

/**
 * @brief Compute XOR checksum over an ASCII payload (between '$' and '*').
 */
uint8_t computeXorChecksum(const std::string& payload);

/**
 * @brief Serialize a ManipulatorCommand into a framed ASCII serial packet:
 *        `$CMD,<dxl_ticks>,<dxl_vel>,<mg996r_us>,<torque>*<HEX_CS>\n`
 */
std::string encodeCommandFrame(const ManipulatorCommand& cmd);

/**
 * @brief Parse a `$STAT,<dxl_ticks>,<dxl_vel>,<load_ma>,<mg996r_us>,<grasped>,<err>*<HEX_CS>`
 *        telemetry line received from the microcontroller.
 */
bool decodeTelemetryFrame(const std::string& line,
                          const ManipulatorCommand& last_cmd,
                          double angle_tolerance_rad,
                          ManipulatorTelemetry& out_telemetry);

/**
 * @brief POSIX Serial (termios) bridge to the Microcontroller (Arduino/ESP32/STM32)
 *        controlling the Dynamixel XL430-W250-T and MG996R servo.
 *        Supports a built-in hardware simulation mode when no physical serial port is connected.
 */
class SerialMcuBridge {
 public:
  SerialMcuBridge(std::string port_name, int baud_rate, bool simulate_if_unavailable = true);
  ~SerialMcuBridge();

  SerialMcuBridge(const SerialMcuBridge&) = delete;
  SerialMcuBridge& operator=(const SerialMcuBridge&) = delete;

  bool connect();
  void disconnect();
  bool isConnected() const { return fd_ >= 0 || simulation_mode_; }
  bool isSimulationMode() const { return simulation_mode_; }

  bool sendCommand(const ManipulatorCommand& cmd);
  bool pollTelemetry(ManipulatorTelemetry& telemetry, double dt_sec = 0.02);

  const ManipulatorCommand& lastCommand() const { return last_cmd_; }

 private:
  void stepSimulation(double dt_sec);

  std::string port_name_;
  int baud_rate_;
  bool simulate_if_unavailable_;
  int fd_{-1};
  bool simulation_mode_{false};
  std::string rx_buffer_;

  ManipulatorCommand last_cmd_{};
  ManipulatorTelemetry sim_state_{};
};

}  // namespace relbot::hw
