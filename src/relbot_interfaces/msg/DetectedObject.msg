# Object detection and 3D relative pose estimate published by Rust perception module
std_msgs/Header header

# Class label (e.g., "pickup_target", "delivery_zone", "obstacle")
string label

# Detection confidence [0.0, 1.0]
float32 confidence

# Normalized bounding box center in image coordinates [-1.0, 1.0]
float32 bbox_center_x
float32 bbox_center_y
float32 bbox_width
float32 bbox_height

# Estimated relative distance and bearing in robot base frame
float64 estimated_distance_m
float64 estimated_bearing_rad
