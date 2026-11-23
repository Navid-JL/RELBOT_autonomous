//! Core data types mirroring `relbot_interfaces` and standard ROS 2 messages (`Twist`, `Odometry`).

/// 2D Robot pose and body-frame velocities from `/odom`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose2D {
    pub x_m: f64,
    pub y_m: f64,
    pub yaw_rad: f64,
}

impl Pose2D {
    pub fn distance_to(&self, other: &Pose2D) -> f64 {
        let dx = other.x_m - self.x_m;
        let dy = other.y_m - self.y_m;
        (dx * dx + dy * dy).sqrt()
    }

    pub fn bearing_to(&self, other: &Pose2D) -> f64 {
        let dx = other.x_m - self.x_m;
        let dy = other.y_m - self.y_m;
        let world_angle = dy.atan2(dx);
        normalize_angle(world_angle - self.yaw_rad)
    }
}

/// Normalize an angle in radians to `[-PI, PI]`.
pub fn normalize_angle(angle_rad: f64) -> f64 {
    let mut a = (angle_rad + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI);
    if a < 0.0 {
        a += 2.0 * std::f64::consts::PI;
    }
    a - std::f64::consts::PI
}

/// Differential-drive velocity command published on `/cmd_vel` (`geometry_msgs/msg/Twist`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TwistCommand {
    pub linear_x_m_s: f64,
    pub angular_z_rad_s: f64,
}

impl TwistCommand {
    pub const fn stop() -> Self {
        Self {
            linear_x_m_s: 0.0,
            angular_z_rad_s: 0.0,
        }
    }
}

/// Command sent to the C++ hardware bridge on `/arm/command` (`relbot_interfaces/msg/ArmCommand`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmCommand {
    /// Target angle for the 1-DOF Dynamixel XL430 arm in radians.
    pub arm_angle_rad: f64,
    /// Profile velocity limit for Dynamixel XL430 in rad/s.
    pub arm_velocity_rad_s: f64,
    /// Target angle for the MG996R gripper servo in degrees.
    pub gripper_angle_deg: f64,
    /// Enable Dynamixel torque.
    pub torque_enable: bool,
}

impl ArmCommand {
    /// Stowed carry pose with gripper open.
    pub const fn stowed_open() -> Self {
        Self {
            arm_angle_rad: 0.10,
            arm_velocity_rad_s: 1.20,
            gripper_angle_deg: 20.0,
            torque_enable: true,
        }
    }

    /// Stowed carry pose while grasping an object.
    pub const fn stowed_closed() -> Self {
        Self {
            arm_angle_rad: 0.10,
            arm_velocity_rad_s: 1.20,
            gripper_angle_deg: 98.0,
            torque_enable: true,
        }
    }
}

/// Telemetry received from the C++ hardware node on `/arm/state` (`relbot_interfaces/msg/ManipulatorState`).
#[derive(Debug, Clone, PartialEq)]
pub struct ManipulatorState {
    pub current_arm_angle_rad: f64,
    pub current_arm_velocity_rad_s: f64,
    pub arm_load_ma: f64,
    pub current_gripper_angle_deg: f64,
    pub target_reached: bool,
    pub object_grasped: bool,
    pub error_code: u8,
}

impl Default for ManipulatorState {
    fn default() -> Self {
        Self {
            current_arm_angle_rad: 0.10,
            current_arm_velocity_rad_s: 0.0,
            arm_load_ma: 100.0,
            current_gripper_angle_deg: 20.0,
            target_reached: true,
            object_grasped: false,
            error_code: 0,
        }
    }
}

/// Semantic class of detected object in the camera frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetClass {
    PickupObject,
    DeliveryZone,
}

impl TargetClass {
    pub fn as_str(&self) -> &'static str {
        match self {
            TargetClass::PickupObject => "pickup_target",
            TargetClass::DeliveryZone => "delivery_zone",
        }
    }
}

/// Output of the Rust object detection & monocular 3D estimation pipeline (`relbot_interfaces/msg/DetectedObject`).
#[derive(Debug, Clone, PartialEq)]
pub struct DetectedObject {
    pub class: TargetClass,
    pub confidence: f32,
    /// Normalized horizontal image center in `[-1.0, 1.0]` (`-1.0` = left edge, `+1.0` = right edge).
    pub bbox_center_x: f32,
    /// Normalized vertical image center in `[-1.0, 1.0]`.
    pub bbox_center_y: f32,
    pub bbox_width: f32,
    pub bbox_height: f32,
    /// Estimated planar distance from the RELBot front pickup point in meters.
    pub estimated_distance_m: f64,
    /// Estimated relative heading error in radians (positive = target is to the left).
    pub estimated_bearing_rad: f64,
}
