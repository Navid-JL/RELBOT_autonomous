#pragma once

#include <cmath>
#include <cstdint>

namespace relbot::hw {

struct WheelVelocities {
  double left_rad_s{0.0};
  double right_rad_s{0.0};
};

struct OdometryPose {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double linear_vel_m_s{0.0};
  double angular_vel_rad_s{0.0};
};

/**
 * @brief Differential-drive kinematics and motor command interface for the RELBot base.
 *
 * Converts body-frame twist commands (v, omega) from the Rust navigation stack into
 * left/right wheel angular velocity setpoints for the RELBot motor controller (XRF2 / FPGA
 * or ROS 2 motor setpoint topics) and integrates wheel encoder ticks into 2D odometry.
 */
class RelbotBaseDriver {
 public:
  RelbotBaseDriver(double wheel_radius_m = 0.050,
                   double track_width_m = 0.210,
                   double max_wheel_rad_s = 12.0);

  /**
   * @brief Inverse kinematics: body twist (m/s, rad/s) -> clamped left/right wheel velocities (rad/s).
   */
  WheelVelocities twistToWheelVelocities(double linear_m_s, double angular_rad_s) const;

  /**
   * @brief Forward kinematics & dead-reckoning odometry integration.
   */
  OdometryPose updateOdometry(double measured_left_rad_s,
                              double measured_right_rad_s,
                              double dt_sec);

  const OdometryPose& pose() const { return pose_; }
  void resetOdometry();

 private:
  double wheel_radius_m_;
  double track_width_m_;
  double max_wheel_rad_s_;
  OdometryPose pose_{};
};

}  // namespace relbot::hw
