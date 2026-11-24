//! C-ABI FFI bindings exposing the Rust perception, visual-servoing controller,
//! and autonomous mission FSM to the C++ ROS 2 Humble nodes on the RELBot Pi.

use std::ffi::c_char;
use std::slice;

use crate::controller::{NavigationConfig, NavigationController};
use crate::perception::{HsvRange, ObjectDetector, PerceptionConfig};
use crate::state_machine::{MissionState, MissionStateMachine};
use crate::types::{DetectedObject, ManipulatorState, Pose2D, TargetClass};

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct RelbotHsvRangeC {
    pub h_min: f32,
    pub h_max: f32,
    pub s_min: f32,
    pub v_min: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct RelbotPerceptionConfigC {
    pub image_width: u32,
    pub image_height: u32,
    pub horizontal_fov_rad: f64,
    pub vertical_fov_rad: f64,
    pub pickup_object_height_m: f64,
    pub delivery_marker_height_m: f64,
    pub min_blob_pixels: u32,
    pub pickup_hsv: RelbotHsvRangeC,
    pub delivery_hsv: RelbotHsvRangeC,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct RelbotNavigationConfigC {
    pub pickup_standoff_m: f64,
    pub pickup_distance_tolerance_m: f64,
    pub pickup_bearing_tolerance_rad: f64,
    pub delivery_standoff_m: f64,
    pub kp_linear: f64,
    pub kp_angular: f64,
    pub max_linear_speed_m_s: f64,
    pub max_angular_speed_rad_s: f64,
    pub search_spin_rad_s: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RelbotDetectedObjectC {
    pub valid: bool,
    /// 0 = PickupObject, 1 = DeliveryZone
    pub target_class: u8,
    pub confidence: f32,
    pub bbox_center_x: f32,
    pub bbox_center_y: f32,
    pub bbox_width: f32,
    pub bbox_height: f32,
    pub estimated_distance_m: f64,
    pub estimated_bearing_rad: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RelbotPose2DC {
    pub x_m: f64,
    pub y_m: f64,
    pub yaw_rad: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct RelbotManipulatorStateC {
    pub current_arm_angle_rad: f64,
    pub current_arm_velocity_rad_s: f64,
    pub arm_load_ma: f64,
    pub current_gripper_angle_deg: f64,
    pub target_reached: bool,
    pub object_grasped: bool,
    pub error_code: u8,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RelbotTwistC {
    pub linear_x_m_s: f64,
    pub angular_z_rad_s: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RelbotMissionOutputC {
    /// Raw MissionState discriminant
    pub state_raw: u8,
    /// Mapped to relbot_interfaces/msg/MissionStatus constants (0..8, 99)
    pub status_code: u8,
    pub cmd_linear_x_m_s: f64,
    pub cmd_angular_z_rad_s: f64,
    pub arm_angle_rad: f64,
    pub arm_velocity_rad_s: f64,
    pub gripper_angle_deg: f64,
    pub torque_enable: bool,
    pub distance_to_goal_m: f64,
    pub holding_object: bool,
}

pub struct RelbotDetectorHandle {
    detector: ObjectDetector,
}

pub struct RelbotMissionHandle {
    fsm: MissionStateMachine,
}

impl From<RelbotHsvRangeC> for HsvRange {
    fn from(c: RelbotHsvRangeC) -> Self {
        Self {
            h_min: c.h_min,
            h_max: c.h_max,
            s_min: c.s_min,
            v_min: c.v_min,
        }
    }
}

impl From<HsvRange> for RelbotHsvRangeC {
    fn from(r: HsvRange) -> Self {
        Self {
            h_min: r.h_min,
            h_max: r.h_max,
            s_min: r.s_min,
            v_min: r.v_min,
        }
    }
}

fn target_class_from_u8(val: u8) -> TargetClass {
    match val {
        1 => TargetClass::DeliveryZone,
        _ => TargetClass::PickupObject,
    }
}

fn detected_to_c(det: DetectedObject) -> RelbotDetectedObjectC {
    RelbotDetectedObjectC {
        valid: true,
        target_class: match det.class {
            TargetClass::PickupObject => 0,
            TargetClass::DeliveryZone => 1,
        },
        confidence: det.confidence,
        bbox_center_x: det.bbox_center_x,
        bbox_center_y: det.bbox_center_y,
        bbox_width: det.bbox_width,
        bbox_height: det.bbox_height,
        estimated_distance_m: det.estimated_distance_m,
        estimated_bearing_rad: det.estimated_bearing_rad,
    }
}

fn c_to_detected(c: &RelbotDetectedObjectC) -> Option<DetectedObject> {
    if !c.valid {
        return None;
    }
    Some(DetectedObject {
        class: target_class_from_u8(c.target_class),
        confidence: c.confidence,
        bbox_center_x: c.bbox_center_x,
        bbox_center_y: c.bbox_center_y,
        bbox_width: c.bbox_width,
        bbox_height: c.bbox_height,
        estimated_distance_m: c.estimated_distance_m,
        estimated_bearing_rad: c.estimated_bearing_rad,
    })
}

fn map_mission_status_code(state: MissionState) -> u8 {
    match state {
        MissionState::Idle => 0,
        MissionState::SearchingTarget => 1,
        MissionState::ApproachingTarget => 2,
        MissionState::AligningForPickup => 3,
        MissionState::PickupOpenGripper
        | MissionState::PickupLowerArm
        | MissionState::PickupCloseGripper
        | MissionState::PickupRaiseArm => 4,
        MissionState::SearchingDeliveryZone => 5,
        MissionState::NavigatingToDelivery => 6,
        MissionState::ReleaseLowerArm
        | MissionState::ReleaseOpenGripper
        | MissionState::ReleaseRaiseArm => 7,
        MissionState::Completed => 8,
        MissionState::Fault => 99,
    }
}

#[no_mangle]
pub extern "C" fn relbot_perception_config_default() -> RelbotPerceptionConfigC {
    let cfg = PerceptionConfig::default();
    RelbotPerceptionConfigC {
        image_width: cfg.image_width,
        image_height: cfg.image_height,
        horizontal_fov_rad: cfg.horizontal_fov_rad,
        vertical_fov_rad: cfg.vertical_fov_rad,
        pickup_object_height_m: cfg.pickup_object_height_m,
        delivery_marker_height_m: cfg.delivery_marker_height_m,
        min_blob_pixels: cfg.min_blob_pixels as u32,
        pickup_hsv: cfg.pickup_hsv.into(),
        delivery_hsv: cfg.delivery_hsv.into(),
    }
}

#[no_mangle]
pub extern "C" fn relbot_navigation_config_default() -> RelbotNavigationConfigC {
    let cfg = NavigationConfig::default();
    RelbotNavigationConfigC {
        pickup_standoff_m: cfg.pickup_standoff_m,
        pickup_distance_tolerance_m: cfg.pickup_distance_tolerance_m,
        pickup_bearing_tolerance_rad: cfg.pickup_bearing_tolerance_rad,
        delivery_standoff_m: cfg.delivery_standoff_m,
        kp_linear: cfg.kp_linear,
        kp_angular: cfg.kp_angular,
        max_linear_speed_m_s: cfg.max_linear_speed_m_s,
        max_angular_speed_rad_s: cfg.max_angular_speed_rad_s,
        search_spin_rad_s: cfg.search_spin_rad_s,
    }
}

#[no_mangle]
pub extern "C" fn relbot_detector_create(
    cfg: *const RelbotPerceptionConfigC,
) -> *mut RelbotDetectorHandle {
    let config = if cfg.is_null() {
        PerceptionConfig::default()
    } else {
        let c = unsafe { *cfg };
        PerceptionConfig {
            image_width: c.image_width,
            image_height: c.image_height,
            horizontal_fov_rad: c.horizontal_fov_rad,
            vertical_fov_rad: c.vertical_fov_rad,
            pickup_object_height_m: c.pickup_object_height_m,
            delivery_marker_height_m: c.delivery_marker_height_m,
            min_blob_pixels: c.min_blob_pixels as usize,
            pickup_hsv: c.pickup_hsv.into(),
            delivery_hsv: c.delivery_hsv.into(),
        }
    };
    Box::into_raw(Box::new(RelbotDetectorHandle {
        detector: ObjectDetector::new(config),
    }))
}

#[no_mangle]
pub extern "C" fn relbot_detector_destroy(handle: *mut RelbotDetectorHandle) {
    if !handle.is_null() {
        unsafe {
            drop(Box::from_raw(handle));
        }
    }
}

#[no_mangle]
pub extern "C" fn relbot_detector_detect_rgb8(
    handle: *const RelbotDetectorHandle,
    rgb_bytes: *const u8,
    rgb_len: usize,
    width: u32,
    height: u32,
    target_class: u8,
) -> RelbotDetectedObjectC {
    if handle.is_null() || rgb_bytes.is_null() || rgb_len == 0 {
        return RelbotDetectedObjectC::default();
    }
    let h = unsafe { &*handle };
    let slice_bytes = unsafe { slice::from_raw_parts(rgb_bytes, rgb_len) };
    match h
        .detector
        .detect_target(slice_bytes, width, height, target_class_from_u8(target_class))
    {
        Some(det) => detected_to_c(det),
        None => RelbotDetectedObjectC::default(),
    }
}

#[no_mangle]
pub extern "C" fn relbot_mission_create(
    cfg: *const RelbotNavigationConfigC,
) -> *mut RelbotMissionHandle {
    let nav_cfg = if cfg.is_null() {
        NavigationConfig::default()
    } else {
        let c = unsafe { *cfg };
        NavigationConfig {
            pickup_standoff_m: c.pickup_standoff_m,
            pickup_distance_tolerance_m: c.pickup_distance_tolerance_m,
            pickup_bearing_tolerance_rad: c.pickup_bearing_tolerance_rad,
            delivery_standoff_m: c.delivery_standoff_m,
            kp_linear: c.kp_linear,
            kp_angular: c.kp_angular,
            max_linear_speed_m_s: c.max_linear_speed_m_s,
            max_angular_speed_rad_s: c.max_angular_speed_rad_s,
            search_spin_rad_s: c.search_spin_rad_s,
        }
    };
    let controller = NavigationController::new(nav_cfg);
    Box::into_raw(Box::new(RelbotMissionHandle {
        fsm: MissionStateMachine::new(controller),
    }))
}

#[no_mangle]
pub extern "C" fn relbot_mission_destroy(handle: *mut RelbotMissionHandle) {
    if !handle.is_null() {
        unsafe {
            drop(Box::from_raw(handle));
        }
    }
}

#[no_mangle]
pub extern "C" fn relbot_mission_step(
    handle: *mut RelbotMissionHandle,
    odom_pose: *const RelbotPose2DC,
    pickup_det: *const RelbotDetectedObjectC,
    delivery_det: *const RelbotDetectedObjectC,
    manip_state: *const RelbotManipulatorStateC,
    dt_sec: f64,
) -> RelbotMissionOutputC {
    if handle.is_null() {
        return RelbotMissionOutputC::default();
    }
    let h = unsafe { &mut *handle };
    let pose = if odom_pose.is_null() {
        Pose2D::default()
    } else {
        let p = unsafe { *odom_pose };
        Pose2D {
            x_m: p.x_m,
            y_m: p.y_m,
            yaw_rad: p.yaw_rad,
        }
    };
    let pickup = if pickup_det.is_null() {
        None
    } else {
        c_to_detected(unsafe { &*pickup_det })
    };
    let delivery = if delivery_det.is_null() {
        None
    } else {
        c_to_detected(unsafe { &*delivery_det })
    };
    let manip = if manip_state.is_null() {
        ManipulatorState::default()
    } else {
        let m = unsafe { *manip_state };
        ManipulatorState {
            current_arm_angle_rad: m.current_arm_angle_rad,
            current_arm_velocity_rad_s: m.current_arm_velocity_rad_s,
            arm_load_ma: m.arm_load_ma,
            current_gripper_angle_deg: m.current_gripper_angle_deg,
            target_reached: m.target_reached,
            object_grasped: m.object_grasped,
            error_code: m.error_code,
        }
    };

    let out = h.fsm.step(&pose, pickup.as_ref(), delivery.as_ref(), &manip, dt_sec);
    RelbotMissionOutputC {
        state_raw: out.state as u8,
        status_code: map_mission_status_code(out.state),
        cmd_linear_x_m_s: out.cmd_vel.linear_x_m_s,
        cmd_angular_z_rad_s: out.cmd_vel.angular_z_rad_s,
        arm_angle_rad: out.arm_cmd.arm_angle_rad,
        arm_velocity_rad_s: out.arm_cmd.arm_velocity_rad_s,
        gripper_angle_deg: out.arm_cmd.gripper_angle_deg,
        torque_enable: out.arm_cmd.torque_enable,
        distance_to_goal_m: out.distance_to_goal_m,
        holding_object: out.holding_object,
    }
}

/// Returns a null-terminated static C string for a raw `MissionState` discriminant.
#[no_mangle]
pub extern "C" fn relbot_mission_state_name(state_raw: u8) -> *const c_char {
    let s: &'static [u8] = match state_raw {
        0 => b"IDLE\0",
        1 => b"SEARCHING_TARGET\0",
        2 => b"APPROACHING_TARGET\0",
        3 => b"ALIGNING_FOR_PICKUP\0",
        4 => b"PICKUP_OPEN_GRIPPER\0",
        5 => b"PICKUP_LOWER_ARM\0",
        6 => b"PICKUP_CLOSE_GRIPPER\0",
        7 => b"PICKUP_RAISE_ARM\0",
        8 => b"SEARCHING_DELIVERY_ZONE\0",
        9 => b"NAVIGATING_TO_DELIVERY\0",
        10 => b"RELEASE_LOWER_ARM\0",
        11 => b"RELEASE_OPEN_GRIPPER\0",
        12 => b"RELEASE_RAISE_ARM\0",
        13 => b"COMPLETED\0",
        _ => b"FAULT\0",
    };
    s.as_ptr() as *const c_char
}

/// Compute visual-servoing twist directly for continuous target following (with deadband).
#[no_mangle]
pub extern "C" fn relbot_controller_follow_target(
    cfg: *const RelbotNavigationConfigC,
    target: *const RelbotDetectedObjectC,
    desired_standoff_m: f64,
    dist_deadband_m: f64,
    bearing_deadband_rad: f64,
) -> RelbotTwistC {
    if target.is_null() {
        return RelbotTwistC::default();
    }
    let Some(det) = c_to_detected(unsafe { &*target }) else {
        return RelbotTwistC::default();
    };
    let nav_cfg = if cfg.is_null() {
        NavigationConfig::default()
    } else {
        let c = unsafe { *cfg };
        NavigationConfig {
            pickup_standoff_m: c.pickup_standoff_m,
            pickup_distance_tolerance_m: c.pickup_distance_tolerance_m,
            pickup_bearing_tolerance_rad: c.pickup_bearing_tolerance_rad,
            delivery_standoff_m: c.delivery_standoff_m,
            kp_linear: c.kp_linear,
            kp_angular: c.kp_angular,
            max_linear_speed_m_s: c.max_linear_speed_m_s,
            max_angular_speed_rad_s: c.max_angular_speed_rad_s,
            search_spin_rad_s: c.search_spin_rad_s,
        }
    };
    let controller = NavigationController::new(nav_cfg);
    let mut twist = controller.compute_visual_servo_twist(&det, desired_standoff_m);

    if (det.estimated_distance_m - desired_standoff_m).abs() <= dist_deadband_m {
        twist.linear_x_m_s = 0.0;
    }
    if det.estimated_bearing_rad.abs() <= bearing_deadband_rad {
        twist.angular_z_rad_s = 0.0;
    }

    RelbotTwistC {
        linear_x_m_s: twist.linear_x_m_s,
        angular_z_rad_s: twist.angular_z_rad_s,
    }
}
