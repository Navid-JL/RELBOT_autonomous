# Telemetry published by the C++ hardware bridge from the microcontroller
std_msgs/Header header

# Current 1-DOF Dynamixel XL430 arm angle in radians
float64 current_arm_angle_rad

# Current Dynamixel XL430 angular velocity in rad/s
float64 current_arm_velocity_rad_s

# Current Dynamixel XL430 load / current estimate (mA or % load)
float64 arm_load

# Current MG996R gripper angle in degrees
float64 current_gripper_angle_deg

# True if arm angle is within tolerance of target and servo has settled
bool target_reached

# True if gripper load / switch indicates an object is grasped
bool object_grasped

# Hardware error code from Dynamixel / MCU bridge (0 = OK)
uint8 error_code
string status_message
