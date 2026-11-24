#ifndef RELBOT_HARDWARE_CPP_RELBOT_NAVIGATION_FFI_H_
#define RELBOT_HARDWARE_CPP_RELBOT_NAVIGATION_FFI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float h_min;
  float h_max;
  float s_min;
  float v_min;
} RelbotHsvRangeC;

typedef struct {
  uint32_t image_width;
  uint32_t image_height;
  double horizontal_fov_rad;
  double vertical_fov_rad;
  double pickup_object_height_m;
  double delivery_marker_height_m;
  uint32_t min_blob_pixels;
  RelbotHsvRangeC pickup_hsv;
  RelbotHsvRangeC delivery_hsv;
} RelbotPerceptionConfigC;

typedef struct {
  double pickup_standoff_m;
  double pickup_distance_tolerance_m;
  double pickup_bearing_tolerance_rad;
  double delivery_standoff_m;
  double kp_linear;
  double kp_angular;
  double max_linear_speed_m_s;
  double max_angular_speed_rad_s;
  double search_spin_rad_s;
} RelbotNavigationConfigC;

typedef struct {
  bool valid;
  uint8_t target_class;  // 0 = PickupObject, 1 = DeliveryZone
  float confidence;
  float bbox_center_x;
  float bbox_center_y;
  float bbox_width;
  float bbox_height;
  double estimated_distance_m;
  double estimated_bearing_rad;
} RelbotDetectedObjectC;

typedef struct {
  double x_m;
  double y_m;
  double yaw_rad;
} RelbotPose2DC;

typedef struct {
  double current_arm_angle_rad;
  double current_arm_velocity_rad_s;
  double arm_load_ma;
  double current_gripper_angle_deg;
  bool target_reached;
  bool object_grasped;
  uint8_t error_code;
} RelbotManipulatorStateC;

typedef struct {
  double linear_x_m_s;
  double angular_z_rad_s;
} RelbotTwistC;

typedef struct {
  uint8_t state_raw;
  uint8_t status_code;
  double cmd_linear_x_m_s;
  double cmd_angular_z_rad_s;
  double arm_angle_rad;
  double arm_velocity_rad_s;
  double gripper_angle_deg;
  bool torque_enable;
  double distance_to_goal_m;
  bool holding_object;
} RelbotMissionOutputC;

typedef struct RelbotDetectorHandle RelbotDetectorHandle;
typedef struct RelbotMissionHandle RelbotMissionHandle;

RelbotPerceptionConfigC relbot_perception_config_default(void);
RelbotNavigationConfigC relbot_navigation_config_default(void);

RelbotDetectorHandle* relbot_detector_create(const RelbotPerceptionConfigC* cfg);
void relbot_detector_destroy(RelbotDetectorHandle* handle);
RelbotDetectedObjectC relbot_detector_detect_rgb8(
    const RelbotDetectorHandle* handle,
    const uint8_t* rgb_bytes,
    size_t rgb_len,
    uint32_t width,
    uint32_t height,
    uint8_t target_class);

RelbotMissionHandle* relbot_mission_create(const RelbotNavigationConfigC* cfg);
void relbot_mission_destroy(RelbotMissionHandle* handle);
RelbotMissionOutputC relbot_mission_step(
    RelbotMissionHandle* handle,
    const RelbotPose2DC* odom_pose,
    const RelbotDetectedObjectC* pickup_det,
    const RelbotDetectedObjectC* delivery_det,
    const RelbotManipulatorStateC* manip_state,
    double dt_sec);

const char* relbot_mission_state_name(uint8_t state_raw);

RelbotTwistC relbot_controller_follow_target(
    const RelbotNavigationConfigC* cfg,
    const RelbotDetectedObjectC* target,
    double desired_standoff_m,
    double dist_deadband_m,
    double bearing_deadband_rad);

#ifdef __cplusplus
}
#endif

#endif  // RELBOT_HARDWARE_CPP_RELBOT_NAVIGATION_FFI_H_
