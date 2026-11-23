#include <cassert>
#include <cmath>
#include <iostream>

#include "relbot_hardware_cpp/manipulator_sequencer.hpp"
#include "relbot_hardware_cpp/relbot_base_driver.hpp"
#include "relbot_hardware_cpp/serial_mcu_bridge.hpp"

using namespace relbot::hw;

int main() {
  // 1. Test Dynamixel XL430-W250-T tick conversions
  assert(radiansToXl430Ticks(0.0) == 2048);
  const uint16_t ticks_90 = radiansToXl430Ticks(M_PI / 2.0);
  assert(ticks_90 == 3072);
  assert(std::abs(xl430TicksToRadians(ticks_90) - (M_PI / 2.0)) < 1e-3);

  // 2. Test MG996R PWM microsecond conversions
  assert(degreesToMg996rPulseUs(0.0) == 500);
  assert(degreesToMg996rPulseUs(180.0) == 2400);
  assert(std::abs(mg996rPulseUsToDegrees(1450) - 90.0) < 1e-2);

  // 3. Test Serial ASCII frame encoding and XOR checksum validation
  ManipulatorCommand cmd;
  cmd.arm_angle_rad = 1.35;
  cmd.arm_velocity_rad_s = 1.2;
  cmd.gripper_angle_deg = 98.0;
  cmd.torque_enable = true;

  const std::string encoded = encodeCommandFrame(cmd);
  assert(encoded.front() == '$');
  assert(encoded.back() == '\n');

  // Construct a synthetic telemetry packet from MCU and decode it
  const std::string stat_payload = "STAT,2928,50,315.5,1534,1,0";
  const uint8_t cs = computeXorChecksum(stat_payload);
  char stat_frame[128];
  std::snprintf(stat_frame, sizeof(stat_frame), "$%s*%02X\n", stat_payload.c_str(), cs);

  ManipulatorTelemetry decoded;
  const bool ok = decodeTelemetryFrame(stat_frame, cmd, 0.05, decoded);
  assert(ok);
  assert(decoded.object_grasped);
  assert(decoded.target_reached);
  assert(decoded.error_code == 0);

  // 4. Test RelbotBaseDriver differential-drive kinematics & odometry
  RelbotBaseDriver base(0.050, 0.210, 12.0);
  const auto wheels = base.twistToWheelVelocities(0.25, 0.0);
  assert(std::abs(wheels.left_rad_s - 5.0) < 1e-6);
  assert(std::abs(wheels.right_rad_s - 5.0) < 1e-6);
  const auto pose = base.updateOdometry(wheels.left_rad_s, wheels.right_rad_s, 1.0);
  assert(std::abs(pose.x_m - 0.25) < 1e-6);

  // 5. Test full Pickup & Release sequence in simulation mode
  SerialMcuBridge bridge("/dev/nonexistent_relbot_mcu", 115200, true);
  assert(bridge.connect());
  assert(bridge.isSimulationMode());

  ManipulatorSequencer sequencer;
  sequencer.startSequence(ManipulatorSequencer::SequenceType::kPickup);

  ManipulatorTelemetry sim_telem;
  for (int step = 0; step < 300 && sequencer.isBusy(); ++step) {
    bridge.pollTelemetry(sim_telem, 0.02);
    const auto seq_cmd = sequencer.update(sim_telem, 0.02);
    bridge.sendCommand(seq_cmd);
  }
  bridge.pollTelemetry(sim_telem, 0.02);
  assert(!sequencer.isBusy());
  assert(sim_telem.object_grasped);

  std::cout << "[PASS] All C++ RELBot hardware & manipulator tests succeeded.\n";
  return 0;
}
