# Command sent from Rust navigation/FSM node to C++ hardware bridge
std_msgs/Header header

# Target angle for the 1-DOF Dynamixel XL430 arm in radians (e.g., 0.0 = stowed/up, 1.4 = lowered for pickup)
float64 arm_angle_rad

# Profile velocity limit for Dynamixel XL430 in rad/s (0.0 = use default)
float64 arm_velocity_rad_s

# Target angle for the MG996R gripper servo in degrees (e.g., 15.0 = open, 95.0 = closed)
float64 gripper_angle_deg

# Enable/disable Dynamixel torque
bool torque_enable
