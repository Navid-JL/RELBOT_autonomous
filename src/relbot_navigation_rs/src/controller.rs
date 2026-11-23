//! Proportional-Derivative (PD) visual servoing and waypoint navigation controller for RELBot.

use crate::types::{DetectedObject, Pose2D, TwistCommand};

#[derive(Debug, Clone)]
pub struct NavigationConfig {
    /// Desired standoff distance from camera/base to object for 1-DOF arm pickup (meters).
    pub pickup_standoff_m: f64,
    /// Distance tolerance for pickup alignment (meters).
    pub pickup_distance_tolerance_m: f64,
    /// Heading tolerance for pickup alignment (radians).
    pub pickup_bearing_tolerance_rad: f64,
    /// Desired standoff distance for delivery drop-off (meters).
    pub delivery_standoff_m: f64,
    /// Linear proportional gain.
    pub kp_linear: f64,
    /// Angular proportional gain.
    pub kp_angular: f64,
    /// Maximum linear speed (m/s).
    pub max_linear_speed_m_s: f64,
    /// Maximum angular speed (rad/s).
    pub max_angular_speed_rad_s: f64,
    /// In-place search angular speed when scanning for target (rad/s).
    pub search_spin_rad_s: f64,
}

impl Default for NavigationConfig {
    fn default() -> Self {
        Self {
            pickup_standoff_m: 0.18,
            pickup_distance_tolerance_m: 0.025,
            pickup_bearing_tolerance_rad: 0.06,
            delivery_standoff_m: 0.22,
            kp_linear: 0.65,
            kp_angular: 1.80,
            max_linear_speed_m_s: 0.25,
            max_angular_speed_rad_s: 1.10,
            search_spin_rad_s: 0.45,
        }
    }
}

pub struct NavigationController {
    config: NavigationConfig,
}

impl NavigationController {
    pub fn new(config: NavigationConfig) -> Self {
        Self { config }
    }

    pub fn config(&self) -> &NavigationConfig {
        &self.config
    }

    /// Check whether a detected target is aligned within the pickup envelope of the 1-DOF arm.
    pub fn is_aligned_for_pickup(&self, target: &DetectedObject) -> bool {
        let dist_err = (target.estimated_distance_m - self.config.pickup_standoff_m).abs();
        let bearing_err = target.estimated_bearing_rad.abs();
        dist_err <= self.config.pickup_distance_tolerance_m
            && bearing_err <= self.config.pickup_bearing_tolerance_rad
    }

    /// Check whether the robot has reached the delivery release zone.
    pub fn is_at_delivery_zone(&self, target: &DetectedObject) -> bool {
        let dist_err = (target.estimated_distance_m - self.config.delivery_standoff_m).abs();
        let bearing_err = target.estimated_bearing_rad.abs();
        dist_err <= self.config.pickup_distance_tolerance_m * 1.5
            && bearing_err <= self.config.pickup_bearing_tolerance_rad * 1.5
    }

    /// Compute a visual-servoing `/cmd_vel` command to approach and center a detected target.
    pub fn compute_visual_servo_twist(
        &self,
        target: &DetectedObject,
        desired_standoff_m: f64,
    ) -> TwistCommand {
        let dist_error = target.estimated_distance_m - desired_standoff_m;
        let bearing_error = target.estimated_bearing_rad;

        let angular_z = (self.config.kp_angular * bearing_error).clamp(
            -self.config.max_angular_speed_rad_s,
            self.config.max_angular_speed_rad_s,
        );

        // Slow down forward motion when heading error is large so the robot turns toward the object first
        let heading_scale = (1.0 - (bearing_error.abs() / 0.8)).clamp(0.0, 1.0);
        let linear_x = (self.config.kp_linear * dist_error * heading_scale).clamp(
            -self.config.max_linear_speed_m_s * 0.5,
            self.config.max_linear_speed_m_s,
        );

        TwistCommand {
            linear_x_m_s: linear_x,
            angular_z_rad_s: angular_z,
        }
    }

    /// Compute an odometry-based waypoint navigation `/cmd_vel` command when driving to a fallback coordinate.
    pub fn compute_waypoint_twist(&self, current: &Pose2D, goal: &Pose2D) -> TwistCommand {
        let dist = current.distance_to(goal);
        if dist <= self.config.pickup_distance_tolerance_m {
            return TwistCommand::stop();
        }

        let bearing = current.bearing_to(goal);
        let angular_z = (self.config.kp_angular * bearing).clamp(
            -self.config.max_angular_speed_rad_s,
            self.config.max_angular_speed_rad_s,
        );
        let heading_scale = (1.0 - (bearing.abs() / 0.9)).clamp(0.0, 1.0);
        let linear_x = (self.config.kp_linear * dist * heading_scale)
            .clamp(0.0, self.config.max_linear_speed_m_s);

        TwistCommand {
            linear_x_m_s: linear_x,
            angular_z_rad_s: angular_z,
        }
    }

    /// Spin in place to search for a target object or delivery beacon.
    pub fn compute_search_twist(&self) -> TwistCommand {
        TwistCommand {
            linear_x_m_s: 0.0,
            angular_z_rad_s: self.config.search_spin_rad_s,
        }
    }
}
