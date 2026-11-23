#pragma once

#include "relbot_hardware_cpp/serial_mcu_bridge.hpp"

namespace relbot::hw {

/**
 * @brief Non-blocking multi-stage pick-and-place state machine for the 1-DOF Dynamixel XL430 arm
 *        and MG996R servo gripper.
 */
class ManipulatorSequencer {
 public:
  enum class SequenceType {
    kNone = 0,
    kHome,
    kPickup,
    kRelease
  };

  enum class Stage {
    kIdle = 0,
    // Pickup stages:
    kPickupOpenGripper,
    kPickupLowerArm,
    kPickupCloseGripper,
    kPickupRaiseArm,
    // Release stages:
    kReleaseLowerArm,
    kReleaseOpenGripper,
    kReleaseRaiseArm,
    // Home stage:
    kHoming,
    kCompleted
  };

  struct Config {
    double arm_stowed_angle_rad{0.10};     // Upright carry / stowed pose
    double arm_pickup_angle_rad{1.35};     // Lowered to ground pickup pose
    double arm_release_angle_rad{1.15};    // Slightly above ground for gentle release
    double arm_speed_rad_s{1.20};          // Controlled XL430 profile velocity
    double gripper_open_deg{20.0};         // MG996R open angle
    double gripper_closed_deg{98.0};       // MG996R closed grasping angle
    double gripper_settle_time_sec{0.45};  // Time allocated for MG996R to open/close
  };

  ManipulatorSequencer();
  explicit ManipulatorSequencer(Config config);

  void startSequence(SequenceType type);
  bool isBusy() const { return stage_ != Stage::kIdle && stage_ != Stage::kCompleted; }
  Stage currentStage() const { return stage_; }

  /**
   * @brief Advance sequence based on latest telemetry and elapsed time dt_sec.
   * @return ManipulatorCommand to send to the SerialMcuBridge.
   */
  ManipulatorCommand update(const ManipulatorTelemetry& telemetry, double dt_sec);

 private:
  Config config_;
  SequenceType active_seq_{SequenceType::kNone};
  Stage stage_{Stage::kIdle};
  double stage_timer_sec_{0.0};
  ManipulatorCommand current_cmd_{};
};

}  // namespace relbot::hw
