use crate::config::ListenerConfigColor;
use crate::ros::ROS;
use nalgebra::geometry::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};
use rclrs::{RclrsError, SubscriptionOptions};
use std::sync::{Arc, RwLock};

pub struct PolygonStampedListener {
    pub config: ListenerConfigColor,
    /// Pre-computed line segments (x1, y1, x2, y2) in fixed-frame world space.
    pub lines: Arc<RwLock<Vec<(f64, f64, f64, f64)>>>,
    /// Polygon points in robot body frame (centered at robot origin, robot-relative orientation).
    /// Used for rendering a ghost footprint at an arbitrary pose.
    pub raw_points: Arc<RwLock<Vec<(f64, f64)>>>,
    _sub: Arc<dyn std::any::Any + Send + Sync>,
}

impl PolygonStampedListener {
    pub fn new(
        config: ListenerConfigColor,
        ros: Arc<ROS>,
        fixed_frame: String,
        robot_frame: String,
    ) -> Result<Self, RclrsError> {
        let lines: Arc<RwLock<Vec<(f64, f64, f64, f64)>>> = Arc::new(RwLock::new(Vec::new()));
        let raw_points: Arc<RwLock<Vec<(f64, f64)>>> = Arc::new(RwLock::new(Vec::new()));
        let lines_cb = Arc::clone(&lines);
        let raw_points_cb = Arc::clone(&raw_points);

        let tf = Arc::clone(&ros.tf);
        let frames = Arc::clone(&ros.frames);

        let sub = ros.node.create_subscription::<geometry_msgs::msg::PolygonStamped, _>(
            SubscriptionOptions::new(&config.topic),
            move |msg: geometry_msgs::msg::PolygonStamped| {
                let pts = &msg.polygon.points;
                if pts.len() < 2 {
                    return;
                }

                let stamp = {
                    let frm = frames.lock().unwrap();
                    frm.values().filter_map(|(_, s)| *s).min()
                };
                let Some(stamp) = stamp else { return };

                // Transform polygon points to fixed frame.
                let tf_iso: Isometry3<f64> = if msg.header.frame_id == fixed_frame {
                    Isometry3::identity()
                } else {
                    let result = tf
                        .lock()
                        .unwrap()
                        .get_transform(&fixed_frame, &msg.header.frame_id, stamp);
                    let Ok(t) = result else { return };
                    let tra =
                        Translation3::new(t.translation.x, t.translation.y, t.translation.z);
                    let rot = UnitQuaternion::new_normalize(Quaternion::new(
                        t.rotation.w,
                        t.rotation.x,
                        t.rotation.y,
                        t.rotation.z,
                    ));
                    Isometry3::from_parts(tra, rot)
                };

                let world_pts: Vec<(f64, f64)> = pts
                    .iter()
                    .map(|p| {
                        let w = tf_iso
                            .transform_point(&Point3::new(p.x as f64, p.y as f64, 0.0));
                        (w.x, w.y)
                    })
                    .collect();

                // Store polygon points in robot body frame so the ghost footprint
                // can be re-positioned at any pose without carrying world-frame offsets.
                let robot_iso: Option<Isometry3<f64>> = {
                    let result = tf
                        .lock()
                        .unwrap()
                        .get_transform(&fixed_frame, &robot_frame, stamp);
                    result.ok().map(|t| {
                        let tra = Translation3::new(
                            t.translation.x,
                            t.translation.y,
                            t.translation.z,
                        );
                        let rot = UnitQuaternion::new_normalize(Quaternion::new(
                            t.rotation.w,
                            t.rotation.x,
                            t.rotation.y,
                            t.rotation.z,
                        ));
                        Isometry3::from_parts(tra, rot)
                    })
                };

                if let Some(robot_world) = robot_iso {
                    let robot_world_inv = robot_world.inverse();
                    *raw_points_cb.write().unwrap() = world_pts
                        .iter()
                        .map(|(wx, wy)| {
                            let bp = robot_world_inv
                                .transform_point(&Point3::new(*wx, *wy, 0.0));
                            (bp.x, bp.y)
                        })
                        .collect();
                }

                let mut new_lines = Vec::new();
                for i in 0..world_pts.len() {
                    let p0 = world_pts[i];
                    let p1 = world_pts[(i + 1) % world_pts.len()];
                    new_lines.push((p0.0, p0.1, p1.0, p1.1));
                }

                *lines_cb.write().unwrap() = new_lines;
            },
        )?;

        Ok(PolygonStampedListener { config, lines, raw_points, _sub: Arc::new(sub) })
    }
}
