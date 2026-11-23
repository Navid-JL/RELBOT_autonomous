#include "relbot_hardware_cpp/relbot_base_driver.hpp"

#include <algorithm>
#include <cmath>

namespace relbot::hw {

RelbotBaseDriver::RelbotBaseDriver(double wheel_radius_m,
                                   double track_width_m,
                                   double max_wheel_rad_s)
    : wheel_radius_m_(wheel_radius_m),
      track_width_m_(track_width_m),
      max_wheel_rad_s_(max_wheel_rad_s) {}

WheelVelocities RelbotBaseDriver::twistToWheelVelocities(double linear_m_s,
                                                         double angular_rad_s) const {
  const double v_left = linear_m_s - 0.5 * track_width_m_ * angular_rad_s;
  const double v_right = linear_m_s + 0.5 * track_width_m_ * angular_rad_s;

  WheelVelocities w;
  w.left_rad_s = std::clamp(v_left / wheel_radius_m_, -max_wheel_rad_s_, max_wheel_rad_s_);
  w.right_rad_s = std::clamp(v_right / wheel_radius_m_, -max_wheel_rad_s_, max_wheel_rad_s_);
  return w;
}

OdometryPose RelbotBaseDriver::updateOdometry(double measured_left_rad_s,
                                              double measured_right_rad_s,
                                              double dt_sec) {
  const double v_left = measured_left_rad_s * wheel_radius_m_;
  const double v_right = measured_right_rad_s * wheel_radius_m_;

  const double v = 0.5 * (v_right + v_left);
  const double omega = (v_right - v_left) / track_width_m_;

  // Midpoint (Runge-Kutta 2nd order) integration
  const double mid_yaw = pose_.yaw_rad + 0.5 * omega * dt_sec;
  pose_.x_m += v * std::cos(mid_yaw) * dt_sec;
  pose_.y_m += v * std::sin(mid_yaw) * dt_sec;
  pose_.yaw_rad = std::atan2(std::sin(pose_.yaw_rad + omega * dt_sec),
                             std::cos(pose_.yaw_rad + omega * dt_sec));
  pose_.linear_vel_m_s = v;
  pose_.angular_vel_rad_s = omega;
  return pose_;
}

void RelbotBaseDriver::resetOdometry() {
  pose_ = OdometryPose{};
}

}  // namespace relbot::hw
