#include "relbot_hardware_cpp/manipulator_sequencer.hpp"

#include <cmath>

namespace relbot::hw {

ManipulatorSequencer::ManipulatorSequencer() : ManipulatorSequencer(Config{}) {}

ManipulatorSequencer::ManipulatorSequencer(Config config) : config_(config) {
  current_cmd_.arm_angle_rad = config_.arm_stowed_angle_rad;
  current_cmd_.arm_velocity_rad_s = config_.arm_speed_rad_s;
  current_cmd_.gripper_angle_deg = config_.gripper_open_deg;
  current_cmd_.torque_enable = true;
}

void ManipulatorSequencer::startSequence(SequenceType type) {
  active_seq_ = type;
  stage_timer_sec_ = 0.0;

  switch (type) {
    case SequenceType::kPickup:
      stage_ = Stage::kPickupOpenGripper;
      break;
    case SequenceType::kRelease:
      stage_ = Stage::kReleaseLowerArm;
      break;
    case SequenceType::kHome:
      stage_ = Stage::kHoming;
      break;
    case SequenceType::kNone:
    default:
      stage_ = Stage::kIdle;
      break;
  }
}

ManipulatorCommand ManipulatorSequencer::update(const ManipulatorTelemetry& telemetry,
                                                double dt_sec) {
  stage_timer_sec_ += dt_sec;
  current_cmd_.arm_velocity_rad_s = config_.arm_speed_rad_s;
  current_cmd_.torque_enable = true;

  auto armReached = [&](double target_rad) {
    return std::abs(telemetry.current_arm_angle_rad - target_rad) <= 0.05;
  };

  switch (stage_) {
    case Stage::kPickupOpenGripper:
      current_cmd_.arm_angle_rad = config_.arm_stowed_angle_rad;
      current_cmd_.gripper_angle_deg = config_.gripper_open_deg;
      if (stage_timer_sec_ >= config_.gripper_settle_time_sec) {
        stage_ = Stage::kPickupLowerArm;
        stage_timer_sec_ = 0.0;
      }
      break;

    case Stage::kPickupLowerArm:
      current_cmd_.arm_angle_rad = config_.arm_pickup_angle_rad;
      current_cmd_.gripper_angle_deg = config_.gripper_open_deg;
      if (armReached(config_.arm_pickup_angle_rad)) {
        stage_ = Stage::kPickupCloseGripper;
        stage_timer_sec_ = 0.0;
      }
      break;

    case Stage::kPickupCloseGripper:
      current_cmd_.arm_angle_rad = config_.arm_pickup_angle_rad;
      current_cmd_.gripper_angle_deg = config_.gripper_closed_deg;
      if (stage_timer_sec_ >= config_.gripper_settle_time_sec) {
        stage_ = Stage::kPickupRaiseArm;
        stage_timer_sec_ = 0.0;
      }
      break;

    case Stage::kPickupRaiseArm:
      current_cmd_.arm_angle_rad = config_.arm_stowed_angle_rad;
      current_cmd_.gripper_angle_deg = config_.gripper_closed_deg;
      if (armReached(config_.arm_stowed_angle_rad)) {
        stage_ = Stage::kCompleted;
        stage_timer_sec_ = 0.0;
      }
      break;

    case Stage::kReleaseLowerArm:
      current_cmd_.arm_angle_rad = config_.arm_release_angle_rad;
      current_cmd_.gripper_angle_deg = config_.gripper_closed_deg;
      if (armReached(config_.arm_release_angle_rad)) {
        stage_ = Stage::kReleaseOpenGripper;
        stage_timer_sec_ = 0.0;
      }
      break;

    case Stage::kReleaseOpenGripper:
      current_cmd_.arm_angle_rad = config_.arm_release_angle_rad;
      current_cmd_.gripper_angle_deg = config_.gripper_open_deg;
      if (stage_timer_sec_ >= config_.gripper_settle_time_sec) {
        stage_ = Stage::kReleaseRaiseArm;
        stage_timer_sec_ = 0.0;
      }
      break;

    case Stage::kReleaseRaiseArm:
      current_cmd_.arm_angle_rad = config_.arm_stowed_angle_rad;
      current_cmd_.gripper_angle_deg = config_.gripper_open_deg;
      if (armReached(config_.arm_stowed_angle_rad)) {
        stage_ = Stage::kCompleted;
        stage_timer_sec_ = 0.0;
      }
      break;

    case Stage::kHoming:
      current_cmd_.arm_angle_rad = config_.arm_stowed_angle_rad;
      current_cmd_.gripper_angle_deg = config_.gripper_open_deg;
      if (armReached(config_.arm_stowed_angle_rad) &&
          stage_timer_sec_ >= config_.gripper_settle_time_sec) {
        stage_ = Stage::kCompleted;
        stage_timer_sec_ = 0.0;
      }
      break;

    case Stage::kCompleted:
    case Stage::kIdle:
    default:
      break;
  }

  return current_cmd_;
}

}  // namespace relbot::hw
