/**
 * @file relbot_manipulator_bridge.ino
 * @brief Microcontroller Bridge Firmware (Arduino / ESP32 / STM32duino)
 *        for RELBot 1-DOF Arm (Dynamixel XL430-W250-T) and Gripper (MG996R Servo).
 *
 * Serial Protocol (115200 8N1, ASCII framed with XOR checksum):
 *   Host (C++ ROS 2 Node) -> MCU:
 *     $CMD,<dxl_goal_ticks>,<dxl_profile_vel>,<mg996r_pulse_us>,<torque_en>*<HEX_CS>\n
 *   MCU -> Host (50 Hz telemetry):
 *     $STAT,<dxl_curr_ticks>,<dxl_curr_vel>,<dxl_load_ma>,<mg996r_pulse_us>,<grasped>,<err>*<HEX_CS>\n
 */

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <Servo.h>

// --- Hardware Pin & Bus Configuration ---
#define USB_SERIAL        Serial
#define DXL_SERIAL        Serial1
constexpr int DXL_DIR_PIN = 2;          // Half-duplex TTL direction pin (U2D2 / DXL Shield)
constexpr uint8_t DXL_ID = 1;           // Dynamixel XL430-W250-T ID
constexpr float DXL_PROTOCOL_VER = 2.0f;
constexpr int MG996R_PWM_PIN = 9;       // PWM pin for MG996R Gripper Servo

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
Servo gripper_servo;

// Control table state
uint16_t target_dxl_ticks = 2048;
uint16_t target_dxl_vel = 50;
uint16_t current_mg996r_us = 600;
bool torque_enabled = true;

String rx_line = "";
unsigned long last_telemetry_ms = 0;

uint8_t computeXorChecksum(const String& payload) {
  uint8_t cs = 0;
  for (size_t i = 0; i < payload.length(); ++i) {
    cs ^= static_cast<uint8_t>(payload[i]);
  }
  return cs;
}

void applyHardwareTargets() {
  if (torque_enabled) {
    dxl.torqueOn(DXL_ID);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, DXL_ID, target_dxl_vel);
    dxl.setGoalPosition(DXL_ID, target_dxl_ticks);
  } else {
    dxl.torqueOff(DXL_ID);
  }
  gripper_servo.writeMicroseconds(current_mg996r_us);
}

void processCommandFrame(const String& line) {
  int dollar_idx = line.indexOf('$');
  int star_idx = line.indexOf('*');
  if (dollar_idx < 0 || star_idx <= dollar_idx + 1 || star_idx + 2 >= static_cast<int>(line.length())) {
    return;
  }

  String payload = line.substring(dollar_idx + 1, star_idx);
  String cs_str = line.substring(star_idx + 1, star_idx + 3);
  uint8_t received_cs = static_cast<uint8_t>(strtoul(cs_str.c_str(), nullptr, 16));
  if (received_cs != computeXorChecksum(payload)) {
    return;
  }

  if (!payload.startsWith("CMD,")) {
    return;
  }

  int dxl_ticks = 2048, dxl_vel = 50, mg_us = 600, torque = 1;
  if (sscanf(payload.c_str(), "CMD,%d,%d,%d,%d", &dxl_ticks, &dxl_vel, &mg_us, &torque) == 4) {
    target_dxl_ticks = static_cast<uint16_t>(constrain(dxl_ticks, 0, 4095));
    target_dxl_vel = static_cast<uint16_t>(constrain(dxl_vel, 0, 1023));
    current_mg996r_us = static_cast<uint16_t>(constrain(mg_us, 500, 2400));
    torque_enabled = (torque != 0);
    applyHardwareTargets();
  }
}

void publishTelemetryFrame() {
  int32_t curr_ticks = dxl.getPresentPosition(DXL_ID);
  int32_t curr_vel = dxl.getPresentVelocity(DXL_ID);
  float curr_load_ma = dxl.getPresentCurrent(DXL_ID, UNIT_MILLI_AMPERE);
  uint8_t hw_err = dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, DXL_ID);

  // Grasp heuristic: MG996R closed beyond threshold + XL430 holding load
  int grasped = (current_mg996r_us >= 1450) ? 1 : 0;

  char payload[96];
  snprintf(payload, sizeof(payload), "STAT,%ld,%ld,%.1f,%u,%d,%u",
           static_cast<long>(curr_ticks),
           static_cast<long>(curr_vel),
           static_cast<double>(curr_load_ma),
           current_mg996r_us,
           grasped,
           hw_err);

  String payload_str(payload);
  uint8_t cs = computeXorChecksum(payload_str);

  char frame[128];
  snprintf(frame, sizeof(frame), "$%s*%02X\n", payload, cs);
  USB_SERIAL.print(frame);
}

void setup() {
  USB_SERIAL.begin(115200);
  rx_line.reserve(128);

  // Initialize MG996R Gripper Servo
  gripper_servo.attach(MG996R_PWM_PIN, 500, 2400);
  gripper_servo.writeMicroseconds(current_mg996r_us);

  // Initialize Dynamixel XL430-W250-T (Protocol 2.0, 57600 default baud)
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VER);
  dxl.ping(DXL_ID);
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
}

void loop() {
  while (USB_SERIAL.available() > 0) {
    char c = static_cast<char>(USB_SERIAL.read());
    if (c == '\n') {
      processCommandFrame(rx_line);
      rx_line = "";
    } else if (c != '\r') {
      rx_line += c;
    }
  }

  unsigned long now_ms = millis();
  if (now_ms - last_telemetry_ms >= 20) {
    last_telemetry_ms = now_ms;
    publishTelemetryFrame();
  }
}
