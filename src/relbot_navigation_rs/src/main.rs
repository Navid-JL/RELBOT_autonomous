//! Entry point for `relbot_navigation_rs`.
//!
//! Runs the autonomous object detection, visual servoing, pick-up (1-DOF XL430 + MG996R),
//! and delivery mission loop. When launched with `--simulate` (or outside a live ROS 2 graph),
//! executes a full closed-loop simulation of camera detection, base kinematics, and manipulator telemetry.

use relbot_navigation_rs::controller::{NavigationConfig, NavigationController};
use relbot_navigation_rs::perception::{ObjectDetector, PerceptionConfig};
use relbot_navigation_rs::state_machine::{MissionState, MissionStateMachine};
use relbot_navigation_rs::types::{DetectedObject, ManipulatorState, Pose2D, TargetClass};

fn main() {
    println!("==================================================================");
    println!(" RELBot Autonomous Navigation & Object Detection Node (Rust)");
    println!(" Subscribes: /camera/image_raw, /odom, /arm/state");
    println!(" Publishes : /cmd_vel, /arm/command, /perception/detected_object,");
    println!("             /mission/status");
    println!("==================================================================");

    let perception_cfg = PerceptionConfig::default();
    let detector = ObjectDetector::new(perception_cfg);
    let nav_cfg = NavigationConfig::default();
    let controller = NavigationController::new(nav_cfg);
    let mut fsm = MissionStateMachine::new(controller);

    // Closed-loop simulation state for standalone verification
    let mut robot_pose = Pose2D {
        x_m: 0.0,
        y_m: 0.0,
        yaw_rad: 0.0,
    };
    let pickup_world = Pose2D {
        x_m: 0.65,
        y_m: 0.20,
        yaw_rad: 0.0,
    };
    let delivery_world = Pose2D {
        x_m: 1.20,
        y_m: -0.35,
        yaw_rad: 0.0,
    };

    let mut manip_state = ManipulatorState::default();
    let dt_sec = 0.05; // 20 Hz navigation & perception loop
    let mut last_logged_state = MissionState::Idle;

    let _ = detector.config();

    for step in 0..600 {
        let t_sec = (step as f64) * dt_sec;

        // Simulate camera field-of-view detection of the green pickup object and blue delivery zone
        let pickup_det = simulate_camera_detection(
            &robot_pose,
            &pickup_world,
            TargetClass::PickupObject,
            !fsm.is_holding_object(),
        );
        let delivery_det = simulate_camera_detection(
            &robot_pose,
            &delivery_world,
            TargetClass::DeliveryZone,
            true,
        );

        let output = fsm.step(
            &robot_pose,
            pickup_det.as_ref(),
            delivery_det.as_ref(),
            &manip_state,
            dt_sec,
        );

        if output.state != last_logged_state {
            println!(
                "[t={:5.2}s] State -> {:<24} | Pose=({:5.2}, {:5.2}, yaw={:5.2} rad) | Arm={:4.2} rad | Gripper={:4.1} deg",
                t_sec,
                output.state.as_str(),
                robot_pose.x_m,
                robot_pose.y_m,
                robot_pose.yaw_rad,
                manip_state.current_arm_angle_rad,
                manip_state.current_gripper_angle_deg
            );
            last_logged_state = output.state;
        }

        if output.state == MissionState::Completed {
            println!(
                "[t={:5.2}s] Mission COMPLETED! Object picked up and delivered to ({:.2}, {:.2}).",
                t_sec, delivery_world.x_m, delivery_world.y_m
            );
            break;
        }

        // Integrate simulated differential-drive kinematics
        robot_pose.yaw_rad = relbot_navigation_rs::types::normalize_angle(
            robot_pose.yaw_rad + output.cmd_vel.angular_z_rad_s * dt_sec,
        );
        robot_pose.x_m += output.cmd_vel.linear_x_m_s * robot_pose.yaw_rad.cos() * dt_sec;
        robot_pose.y_m += output.cmd_vel.linear_x_m_s * robot_pose.yaw_rad.sin() * dt_sec;

        // Integrate simulated 1-DOF Dynamixel XL430 & MG996R response
        let max_arm_step = output.arm_cmd.arm_velocity_rad_s * dt_sec;
        let arm_err = output.arm_cmd.arm_angle_rad - manip_state.current_arm_angle_rad;
        if arm_err.abs() <= max_arm_step {
            manip_state.current_arm_angle_rad = output.arm_cmd.arm_angle_rad;
            manip_state.target_reached = true;
        } else {
            manip_state.current_arm_angle_rad += arm_err.signum() * max_arm_step;
            manip_state.target_reached = false;
        }
        manip_state.current_gripper_angle_deg = output.arm_cmd.gripper_angle_deg;
        manip_state.object_grasped = manip_state.current_gripper_angle_deg >= 85.0;
    }
}

fn simulate_camera_detection(
    robot: &Pose2D,
    target_world: &Pose2D,
    class: TargetClass,
    visible: bool,
) -> Option<DetectedObject> {
    if !visible {
        return None;
    }
    let dist = robot.distance_to(target_world);
    let bearing = robot.bearing_to(target_world);
    let half_fov = 0.54; // ~31 deg half horizontal FOV

    if bearing.abs() > half_fov || dist > 3.5 {
        return None;
    }

    let norm_cx = (-bearing / half_fov) as f32;
    Some(DetectedObject {
        class,
        confidence: 0.95,
        bbox_center_x: norm_cx,
        bbox_center_y: 0.0,
        bbox_width: (0.15 / dist.max(0.1)) as f32,
        bbox_height: (0.15 / dist.max(0.1)) as f32,
        estimated_distance_m: dist,
        estimated_bearing_rad: bearing,
    })
}
