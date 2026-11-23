//! Autonomous Pick-Up and Delivery Finite State Machine (FSM).
//!
//! Coordinates:
//! - Searching & visual-servoing toward the target object
//! - Fine alignment for the 1-DOF Dynamixel XL430 arm pickup envelope
//! - Commanding the C++ hardware bridge (`ArmCommand` / `ExecutePickPlace`) to lower the XL430 arm,
//!   close the MG996R gripper, and raise the arm to the stowed carry pose
//! - Navigating to the delivery zone (via visual beacon or odometry waypoint)
//! - Lowering the arm, opening the MG996R gripper to release the object, and stowing the arm.

use crate::controller::NavigationController;
use crate::types::{
    ArmCommand, DetectedObject, ManipulatorState, Pose2D, TargetClass, TwistCommand,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionState {
    Idle = 0,
    SearchingTarget = 1,
    ApproachingTarget = 2,
    AligningForPickup = 3,
    PickupOpenGripper = 4,
    PickupLowerArm = 5,
    PickupCloseGripper = 6,
    PickupRaiseArm = 7,
    SearchingDeliveryZone = 8,
    NavigatingToDelivery = 9,
    ReleaseLowerArm = 10,
    ReleaseOpenGripper = 11,
    ReleaseRaiseArm = 12,
    Completed = 13,
    Fault = 99,
}

impl MissionState {
    pub fn as_str(&self) -> &'static str {
        match self {
            MissionState::Idle => "IDLE",
            MissionState::SearchingTarget => "SEARCHING_TARGET",
            MissionState::ApproachingTarget => "APPROACHING_TARGET",
            MissionState::AligningForPickup => "ALIGNING_FOR_PICKUP",
            MissionState::PickupOpenGripper => "PICKUP_OPEN_GRIPPER",
            MissionState::PickupLowerArm => "PICKUP_LOWER_ARM",
            MissionState::PickupCloseGripper => "PICKUP_CLOSE_GRIPPER",
            MissionState::PickupRaiseArm => "PICKUP_RAISE_ARM",
            MissionState::SearchingDeliveryZone => "SEARCHING_DELIVERY_ZONE",
            MissionState::NavigatingToDelivery => "NAVIGATING_TO_DELIVERY",
            MissionState::ReleaseLowerArm => "RELEASE_LOWER_ARM",
            MissionState::ReleaseOpenGripper => "RELEASE_OPEN_GRIPPER",
            MissionState::ReleaseRaiseArm => "RELEASE_RAISE_ARM",
            MissionState::Completed => "COMPLETED",
            MissionState::Fault => "FAULT",
        }
    }
}

/// Output produced on each tick of the autonomous mission FSM.
#[derive(Debug, Clone)]
pub struct MissionControlOutput {
    pub state: MissionState,
    pub cmd_vel: TwistCommand,
    pub arm_cmd: ArmCommand,
    pub distance_to_goal_m: f64,
    pub holding_object: bool,
}

pub struct MissionStateMachine {
    state: MissionState,
    controller: NavigationController,
    stage_timer_sec: f64,
    holding_object: bool,
    fallback_delivery_pose: Option<Pose2D>,
    // 1-DOF XL430 & MG996R calibration setpoints
    arm_stowed_rad: f64,
    arm_pickup_rad: f64,
    arm_release_rad: f64,
    arm_speed_rad_s: f64,
    gripper_open_deg: f64,
    gripper_closed_deg: f64,
    gripper_settle_sec: f64,
}

impl MissionStateMachine {
    pub fn new(controller: NavigationController) -> Self {
        Self {
            state: MissionState::SearchingTarget,
            controller,
            stage_timer_sec: 0.0,
            holding_object: false,
            fallback_delivery_pose: None,
            arm_stowed_rad: 0.10,
            arm_pickup_rad: 1.35,
            arm_release_rad: 1.15,
            arm_speed_rad_s: 1.20,
            gripper_open_deg: 20.0,
            gripper_closed_deg: 98.0,
            gripper_settle_sec: 0.45,
        }
    }

    pub fn with_fallback_delivery_pose(mut self, pose: Pose2D) -> Self {
        self.fallback_delivery_pose = Some(pose);
        self
    }

    pub fn state(&self) -> MissionState {
        self.state
    }

    pub fn is_holding_object(&self) -> bool {
        self.holding_object
    }

    /// Advance the mission FSM by `dt_sec`.
    pub fn step(
        &mut self,
        odom_pose: &Pose2D,
        pickup_detection: Option<&DetectedObject>,
        delivery_detection: Option<&DetectedObject>,
        manip_state: &ManipulatorState,
        dt_sec: f64,
    ) -> MissionControlOutput {
        self.stage_timer_sec += dt_sec;

        if manip_state.error_code != 0 {
            self.state = MissionState::Fault;
        }

        let mut cmd_vel = TwistCommand::stop();
        let mut arm_cmd = if self.holding_object {
            ArmCommand::stowed_closed()
        } else {
            ArmCommand::stowed_open()
        };
        let mut distance_to_goal_m = 0.0;

        let arm_at = |target_rad: f64| -> bool {
            (manip_state.current_arm_angle_rad - target_rad).abs() <= 0.05
        };

        match self.state {
            MissionState::Idle => {
                self.state = MissionState::SearchingTarget;
                self.stage_timer_sec = 0.0;
            }

            MissionState::SearchingTarget => {
                if let Some(det) = pickup_detection {
                    debug_assert_eq!(det.class, TargetClass::PickupObject);
                    self.state = MissionState::ApproachingTarget;
                    self.stage_timer_sec = 0.0;
                } else {
                    cmd_vel = self.controller.compute_search_twist();
                }
            }

            MissionState::ApproachingTarget => {
                if let Some(det) = pickup_detection {
                    distance_to_goal_m = det.estimated_distance_m;
                    if self.controller.is_aligned_for_pickup(det) {
                        self.state = MissionState::AligningForPickup;
                        self.stage_timer_sec = 0.0;
                    } else {
                        let standoff = self.controller.config().pickup_standoff_m;
                        cmd_vel = self.controller.compute_visual_servo_twist(det, standoff);
                    }
                } else {
                    self.state = MissionState::SearchingTarget;
                    self.stage_timer_sec = 0.0;
                }
            }

            MissionState::AligningForPickup => {
                if let Some(det) = pickup_detection {
                    distance_to_goal_m = det.estimated_distance_m;
                    if self.controller.is_aligned_for_pickup(det) {
                        // Base is aligned and stopped; begin 1-DOF arm pickup sequence
                        cmd_vel = TwistCommand::stop();
                        self.state = MissionState::PickupOpenGripper;
                        self.stage_timer_sec = 0.0;
                    } else {
                        let standoff = self.controller.config().pickup_standoff_m;
                        cmd_vel = self.controller.compute_visual_servo_twist(det, standoff);
                    }
                } else {
                    self.state = MissionState::SearchingTarget;
                    self.stage_timer_sec = 0.0;
                }
            }

            // --- 1-DOF Dynamixel XL430 & MG996R Pickup Sequence ---
            MissionState::PickupOpenGripper => {
                arm_cmd = ArmCommand {
                    arm_angle_rad: self.arm_stowed_rad,
                    arm_velocity_rad_s: self.arm_speed_rad_s,
                    gripper_angle_deg: self.gripper_open_deg,
                    torque_enable: true,
                };
                if self.stage_timer_sec >= self.gripper_settle_sec {
                    self.state = MissionState::PickupLowerArm;
                    self.stage_timer_sec = 0.0;
                }
            }

            MissionState::PickupLowerArm => {
                arm_cmd = ArmCommand {
                    arm_angle_rad: self.arm_pickup_rad,
                    arm_velocity_rad_s: self.arm_speed_rad_s,
                    gripper_angle_deg: self.gripper_open_deg,
                    torque_enable: true,
                };
                if arm_at(self.arm_pickup_rad) {
                    self.state = MissionState::PickupCloseGripper;
                    self.stage_timer_sec = 0.0;
                }
            }

            MissionState::PickupCloseGripper => {
                arm_cmd = ArmCommand {
                    arm_angle_rad: self.arm_pickup_rad,
                    arm_velocity_rad_s: self.arm_speed_rad_s,
                    gripper_angle_deg: self.gripper_closed_deg,
                    torque_enable: true,
                };
                if self.stage_timer_sec >= self.gripper_settle_sec {
                    self.holding_object = true;
                    self.state = MissionState::PickupRaiseArm;
                    self.stage_timer_sec = 0.0;
                }
            }

            MissionState::PickupRaiseArm => {
                arm_cmd = ArmCommand {
                    arm_angle_rad: self.arm_stowed_rad,
                    arm_velocity_rad_s: self.arm_speed_rad_s,
                    gripper_angle_deg: self.gripper_closed_deg,
                    torque_enable: true,
                };
                if arm_at(self.arm_stowed_rad) {
                    self.state = MissionState::SearchingDeliveryZone;
                    self.stage_timer_sec = 0.0;
                }
            }

            // --- Delivery Navigation Phase ---
            MissionState::SearchingDeliveryZone => {
                if delivery_detection.is_some() || self.fallback_delivery_pose.is_some() {
                    self.state = MissionState::NavigatingToDelivery;
                    self.stage_timer_sec = 0.0;
                } else {
                    cmd_vel = self.controller.compute_search_twist();
                }
            }

            MissionState::NavigatingToDelivery => {
                if let Some(det) = delivery_detection {
                    distance_to_goal_m = det.estimated_distance_m;
                    if self.controller.is_at_delivery_zone(det) {
                        cmd_vel = TwistCommand::stop();
                        self.state = MissionState::ReleaseLowerArm;
                        self.stage_timer_sec = 0.0;
                    } else {
                        let standoff = self.controller.config().delivery_standoff_m;
                        cmd_vel = self.controller.compute_visual_servo_twist(det, standoff);
                    }
                } else if let Some(goal_pose) = &self.fallback_delivery_pose {
                    distance_to_goal_m = odom_pose.distance_to(goal_pose);
                    cmd_vel = self.controller.compute_waypoint_twist(odom_pose, goal_pose);
                    if distance_to_goal_m
                        <= self.controller.config().pickup_distance_tolerance_m
                    {
                        self.state = MissionState::ReleaseLowerArm;
                        self.stage_timer_sec = 0.0;
                    }
                } else {
                    self.state = MissionState::SearchingDeliveryZone;
                    self.stage_timer_sec = 0.0;
                }
            }

            // --- 1-DOF Dynamixel XL430 & MG996R Release Sequence ---
            MissionState::ReleaseLowerArm => {
                arm_cmd = ArmCommand {
                    arm_angle_rad: self.arm_release_rad,
                    arm_velocity_rad_s: self.arm_speed_rad_s,
                    gripper_angle_deg: self.gripper_closed_deg,
                    torque_enable: true,
                };
                if arm_at(self.arm_release_rad) {
                    self.state = MissionState::ReleaseOpenGripper;
                    self.stage_timer_sec = 0.0;
                }
            }

            MissionState::ReleaseOpenGripper => {
                arm_cmd = ArmCommand {
                    arm_angle_rad: self.arm_release_rad,
                    arm_velocity_rad_s: self.arm_speed_rad_s,
                    gripper_angle_deg: self.gripper_open_deg,
                    torque_enable: true,
                };
                if self.stage_timer_sec >= self.gripper_settle_sec {
                    self.holding_object = false;
                    self.state = MissionState::ReleaseRaiseArm;
                    self.stage_timer_sec = 0.0;
                }
            }

            MissionState::ReleaseRaiseArm => {
                arm_cmd = ArmCommand {
                    arm_angle_rad: self.arm_stowed_rad,
                    arm_velocity_rad_s: self.arm_speed_rad_s,
                    gripper_angle_deg: self.gripper_open_deg,
                    torque_enable: true,
                };
                if arm_at(self.arm_stowed_rad) {
                    self.state = MissionState::Completed;
                    self.stage_timer_sec = 0.0;
                }
            }

            MissionState::Completed | MissionState::Fault => {
                cmd_vel = TwistCommand::stop();
            }
        }

        MissionControlOutput {
            state: self.state,
            cmd_vel,
            arm_cmd,
            distance_to_goal_m,
            holding_object: self.holding_object,
        }
    }
}
