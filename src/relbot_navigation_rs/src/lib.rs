pub mod controller;
pub mod perception;
pub mod state_machine;
pub mod types;

#[cfg(test)]
mod tests {
    use crate::controller::{NavigationConfig, NavigationController};
    use crate::perception::{ObjectDetector, PerceptionConfig};
    use crate::state_machine::{MissionState, MissionStateMachine};
    use crate::types::{ManipulatorState, Pose2D, TargetClass};

    /// Helper to render a synthetic RGB8 camera frame containing a colored target rectangle.
    fn render_synthetic_rgb(
        width: u32,
        height: u32,
        rect_x: (u32, u32),
        rect_y: (u32, u32),
        rgb: [u8; 3],
    ) -> Vec<u8> {
        let mut buf = vec![20u8; (width * height * 3) as usize];
        for y in rect_y.0..rect_y.1.min(height) {
            for x in rect_x.0..rect_x.1.min(width) {
                let idx = ((y * width + x) as usize) * 3;
                buf[idx] = rgb[0];
                buf[idx + 1] = rgb[1];
                buf[idx + 2] = rgb[2];
            }
        }
        buf
    }

    #[test]
    fn test_object_detection_and_pose_estimation() {
        let detector = ObjectDetector::new(PerceptionConfig::default());
        // Render a bright green pickup block slightly right of center
        let frame = render_synthetic_rgb(320, 240, (165, 205), (90, 150), [30, 220, 60]);

        let det = detector
            .detect_target(&frame, 320, 240, TargetClass::PickupObject)
            .expect("Expected green pickup block to be detected");

        assert_eq!(det.class, TargetClass::PickupObject);
        assert!(det.confidence > 0.8);
        assert!(det.bbox_center_x > 0.0);
        assert!(det.estimated_bearing_rad < 0.0); // Target to the right => negative bearing
        assert!(det.estimated_distance_m > 0.10 && det.estimated_distance_m < 1.0);
    }

    #[test]
    fn test_full_pickup_and_delivery_mission_fsm() {
        let nav_cfg = NavigationConfig::default();
        let controller = NavigationController::new(nav_cfg);
        let mut fsm = MissionStateMachine::new(controller);

        let mut odom = Pose2D::default();
        let mut manip = ManipulatorState::default();
        let detector = ObjectDetector::new(PerceptionConfig::default());

        // 1. No target visible -> SearchingTarget spins in place
        let out = fsm.step(&odom, None, None, &manip, 0.05);
        assert_eq!(out.state, MissionState::SearchingTarget);
        assert!(out.cmd_vel.angular_z_rad_s > 0.0);

        // 2. Target appears directly ahead at ~0.18m standoff (box height ~92 px)
        let aligned_green_frame =
            render_synthetic_rgb(320, 240, (140, 180), (74, 166), [30, 220, 60]);
        let pickup_det = detector
            .detect_target(&aligned_green_frame, 320, 240, TargetClass::PickupObject)
            .unwrap();

        // Advance through Approaching -> Aligning -> Pickup sequence
        for _ in 0..80 {
            let step_out = fsm.step(&odom, Some(&pickup_det), None, &manip, 0.05);
            // Simulate C++ hardware node tracking the commanded arm & gripper angles
            manip.current_arm_angle_rad = step_out.arm_cmd.arm_angle_rad;
            manip.current_gripper_angle_deg = step_out.arm_cmd.gripper_angle_deg;
            manip.object_grasped = manip.current_gripper_angle_deg >= 85.0;
            if step_out.state == MissionState::SearchingDeliveryZone {
                break;
            }
        }

        assert_eq!(fsm.state(), MissionState::SearchingDeliveryZone);
        assert!(fsm.is_holding_object());

        // 3. Blue delivery zone marker appears ahead at ~0.22m standoff (box height ~139 px)
        let aligned_blue_frame =
            render_synthetic_rgb(320, 240, (135, 185), (50, 189), [30, 80, 235]);
        let delivery_det = detector
            .detect_target(&aligned_blue_frame, 320, 240, TargetClass::DeliveryZone)
            .unwrap();

        for _ in 0..80 {
            let step_out = fsm.step(&odom, None, Some(&delivery_det), &manip, 0.05);
            odom.x_m += step_out.cmd_vel.linear_x_m_s * 0.05;
            manip.current_arm_angle_rad = step_out.arm_cmd.arm_angle_rad;
            manip.current_gripper_angle_deg = step_out.arm_cmd.gripper_angle_deg;
            manip.object_grasped = manip.current_gripper_angle_deg >= 85.0;
            if step_out.state == MissionState::Completed {
                break;
            }
        }

        assert_eq!(fsm.state(), MissionState::Completed);
        assert!(!fsm.is_holding_object());
    }
}
