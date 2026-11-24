//! Object detection and monocular distance/bearing estimation in Rust.
//!
//! Operates on raw RGB8 camera frames (`sensor_msgs/msg/Image`) to detect:
//! 1. `TargetClass::PickupObject` (configurable HSV color segmentation + bounding-box morphology)
//! 2. `TargetClass::DeliveryZone` (delivery drop-off beacon/pad)
//!
//! Also provides pinhole-camera range & bearing estimation from known physical object height
//! and vertical pixel span.

use crate::types::{DetectedObject, TargetClass};

/// HSV color threshold range (`h` in `[0..360]`, `s` and `v` in `[0.0..1.0]`).
#[derive(Debug, Clone, Copy)]
pub struct HsvRange {
    pub h_min: f32,
    pub h_max: f32,
    pub s_min: f32,
    pub v_min: f32,
}

/// Camera intrinsics and object physical dimensions for range/bearing projection.
#[derive(Debug, Clone)]
pub struct PerceptionConfig {
    pub image_width: u32,
    pub image_height: u32,
    /// Horizontal field of view of the RELBot camera in radians (default ~1.08 rad / 62 deg).
    pub horizontal_fov_rad: f64,
    /// Vertical field of view in radians (default ~0.85 rad / 48.8 deg).
    pub vertical_fov_rad: f64,
    /// Known physical height of the pickup target in meters (e.g., 0.065 m cylinder/cube).
    pub pickup_object_height_m: f64,
    /// Known physical height of the delivery zone marker in meters (e.g., 0.12 m beacon).
    pub delivery_marker_height_m: f64,
    /// Minimum matching pixels required to form a valid detection.
    pub min_blob_pixels: usize,
    /// HSV threshold for the pickup object (default: bright green/teal or red block).
    pub pickup_hsv: HsvRange,
    /// HSV threshold for the delivery zone marker (default: blue drop-off zone).
    pub delivery_hsv: HsvRange,
}

impl Default for PerceptionConfig {
    fn default() -> Self {
        Self {
            image_width: 320,
            image_height: 240,
            horizontal_fov_rad: 1.082,
            vertical_fov_rad: 0.850,
            pickup_object_height_m: 0.065,
            delivery_marker_height_m: 0.120,
            min_blob_pixels: 25,
            // Green pickup block
            pickup_hsv: HsvRange {
                h_min: 95.0,
                h_max: 155.0,
                s_min: 0.45,
                v_min: 0.35,
            },
            // Blue delivery zone beacon
            delivery_hsv: HsvRange {
                h_min: 200.0,
                h_max: 255.0,
                s_min: 0.45,
                v_min: 0.35,
            },
        }
    }
}

pub struct ObjectDetector {
    config: PerceptionConfig,
}

impl ObjectDetector {
    pub fn new(config: PerceptionConfig) -> Self {
        Self { config }
    }

    pub fn config(&self) -> &PerceptionConfig {
        &self.config
    }

    /// Detect a specific target class (`PickupObject` or `DeliveryZone`) in an RGB8 buffer.
    pub fn detect_target(
        &self,
        rgb_bytes: &[u8],
        width: u32,
        height: u32,
        target_class: TargetClass,
    ) -> Option<DetectedObject> {
        let expected_len = (width as usize) * (height as usize) * 3;
        if rgb_bytes.len() < expected_len || width == 0 || height == 0 {
            return None;
        }

        let (hsv_range, physical_height_m) = match target_class {
            TargetClass::PickupObject => (
                &self.config.pickup_hsv,
                self.config.pickup_object_height_m,
            ),
            TargetClass::DeliveryZone => (
                &self.config.delivery_hsv,
                self.config.delivery_marker_height_m,
            ),
        };

        let w = width as usize;
        let h = height as usize;
        let total_px = w * h;
        let mut mask = vec![0u8; total_px];
        let mut any_match = false;

        for y in 0..h {
            for x in 0..w {
                let p_idx = y * w + x;
                let idx = p_idx * 3;
                let r = rgb_bytes[idx];
                let g = rgb_bytes[idx + 1];
                let b = rgb_bytes[idx + 2];

                let (hue, s, v) = rgb_to_hsv(r, g, b);
                let hue_ok = if hsv_range.h_min <= hsv_range.h_max {
                    hue >= hsv_range.h_min && hue <= hsv_range.h_max
                } else {
                    // Wrap-around hue range (e.g., red: 345..15 deg)
                    hue >= hsv_range.h_min || hue <= hsv_range.h_max
                };
                if hue_ok && s >= hsv_range.s_min && v >= hsv_range.v_min {
                    mask[p_idx] = 1;
                    any_match = true;
                }
            }
        }

        if !any_match {
            return None;
        }

        // Extract the largest 4-connected component to reject scattered background noise.
        let mut best_count: usize = 0;
        let mut best_sum_x: u64 = 0;
        let mut best_sum_y: u64 = 0;
        let mut best_min_x: u32 = width;
        let mut best_max_x: u32 = 0;
        let mut best_min_y: u32 = height;
        let mut best_max_y: u32 = 0;

        let mut stack: Vec<usize> = Vec::with_capacity(256);

        for start_idx in 0..total_px {
            if mask[start_idx] != 1 {
                continue;
            }
            mask[start_idx] = 2; // visited
            stack.clear();
            stack.push(start_idx);

            let mut count: usize = 0;
            let mut sum_x: u64 = 0;
            let mut sum_y: u64 = 0;
            let mut min_x: u32 = width;
            let mut max_x: u32 = 0;
            let mut min_y: u32 = height;
            let mut max_y: u32 = 0;

            while let Some(curr) = stack.pop() {
                let cx = (curr % w) as u32;
                let cy = (curr / w) as u32;
                count += 1;
                sum_x += cx as u64;
                sum_y += cy as u64;
                if cx < min_x {
                    min_x = cx;
                }
                if cx > max_x {
                    max_x = cx;
                }
                if cy < min_y {
                    min_y = cy;
                }
                if cy > max_y {
                    max_y = cy;
                }

                if cx > 0 {
                    let left = curr - 1;
                    if mask[left] == 1 {
                        mask[left] = 2;
                        stack.push(left);
                    }
                }
                if (cx as usize) + 1 < w {
                    let right = curr + 1;
                    if mask[right] == 1 {
                        mask[right] = 2;
                        stack.push(right);
                    }
                }
                if cy > 0 {
                    let up = curr - w;
                    if mask[up] == 1 {
                        mask[up] = 2;
                        stack.push(up);
                    }
                }
                if (cy as usize) + 1 < h {
                    let down = curr + w;
                    if mask[down] == 1 {
                        mask[down] = 2;
                        stack.push(down);
                    }
                }
            }

            if count > best_count {
                best_count = count;
                best_sum_x = sum_x;
                best_sum_y = sum_y;
                best_min_x = min_x;
                best_max_x = max_x;
                best_min_y = min_y;
                best_max_y = max_y;
            }
        }

        if best_count < self.config.min_blob_pixels {
            return None;
        }

        let count = best_count;
        let sum_x = best_sum_x;
        let sum_y = best_sum_y;
        let min_x = best_min_x;
        let max_x = best_max_x;
        let min_y = best_min_y;
        let max_y = best_max_y;

        let cx_px = (sum_x as f64) / (count as f64);
        let cy_px = (sum_y as f64) / (count as f64);
        let box_w_px = ((max_x - min_x + 1) as f64).max(1.0);
        let box_h_px = ((max_y - min_y + 1) as f64).max(1.0);

        // Normalized image coordinates in [-1.0, 1.0]
        let norm_cx = ((cx_px / (width as f64)) * 2.0 - 1.0) as f32;
        let norm_cy = ((cy_px / (height as f64)) * 2.0 - 1.0) as f32;
        let norm_w = (box_w_px / (width as f64)) as f32;
        let norm_h = (box_h_px / (height as f64)) as f32;

        // Pinhole focal length in pixels: f_y = (H / 2) / tan(VFOV / 2)
        let focal_y_px =
            (height as f64 * 0.5) / (self.config.vertical_fov_rad * 0.5).tan();
        let estimated_distance_m =
            ((physical_height_m * focal_y_px) / box_h_px).clamp(0.08, 5.0);

        // Bearing angle in robot frame: positive when target is left of image center (norm_cx < 0)
        let estimated_bearing_rad =
            -(norm_cx as f64) * (self.config.horizontal_fov_rad * 0.5);

        let box_area = box_w_px * box_h_px;
        let fill_ratio = ((count as f64) / box_area).clamp(0.1, 1.0) as f32;

        Some(DetectedObject {
            class: target_class,
            confidence: fill_ratio,
            bbox_center_x: norm_cx,
            bbox_center_y: norm_cy,
            bbox_width: norm_w,
            bbox_height: norm_h,
            estimated_distance_m,
            estimated_bearing_rad,
        })
    }
}

/// Convert 8-bit RGB to `(Hue [0..360], Saturation [0..1], Value [0..1])`.
pub fn rgb_to_hsv(r: u8, g: u8, b: u8) -> (f32, f32, f32) {
    let rf = (r as f32) / 255.0;
    let gf = (g as f32) / 255.0;
    let bf = (b as f32) / 255.0;

    let max = rf.max(gf).max(bf);
    let min = rf.min(gf).min(bf);
    let delta = max - min;

    let h = if delta < 1e-5 {
        0.0
    } else if (max - rf).abs() < 1e-5 {
        60.0 * (((gf - bf) / delta).rem_euclid(6.0))
    } else if (max - gf).abs() < 1e-5 {
        60.0 * (((bf - rf) / delta) + 2.0)
    } else {
        60.0 * (((rf - gf) / delta) + 4.0)
    };

    let s = if max < 1e-5 { 0.0 } else { delta / max };
    (h, s, max)
}
