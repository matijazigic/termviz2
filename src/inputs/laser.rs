use crate::config::ListenerConfigColor;
use crate::ros::ROS;
use nalgebra::geometry::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};
use rclrs::{RclrsError, SubscriptionOptions};
use std::sync::{Arc, RwLock};

pub struct LaserListener {
    pub config: ListenerConfigColor,
    /// Pre-computed world-space (x, y) points ready for rendering.
    pub points: Arc<RwLock<Vec<(f64, f64)>>>,
    _sub: Arc<dyn std::any::Any + Send + Sync>,
}

impl LaserListener {
    pub fn new(
        config: ListenerConfigColor,
        ros: Arc<ROS>,
        fixed_frame: String,
    ) -> Result<Self, RclrsError> {
        let points: Arc<RwLock<Vec<(f64, f64)>>> = Arc::new(RwLock::new(Vec::new()));
        let points_cb = Arc::clone(&points);

        let tf = Arc::clone(&ros.tf);
        let frames = Arc::clone(&ros.frames);

        let sub_options = SubscriptionOptions::new(&config.topic);
        let sub = ros.node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
            sub_options,
            move |msg: sensor_msgs::msg::LaserScan| {
                let stamp = {
                    let frm = frames.lock().unwrap();
                    frm.values().filter_map(|(_, s)| *s).min()
                };
                let Some(stamp) = stamp else { return };

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

                let mut new_points = Vec::new();
                for (i, &range) in msg.ranges.iter().enumerate() {
                    if range <= msg.range_min || range >= msg.range_max {
                        continue;
                    }
                    let angle = msg.angle_min + i as f32 * msg.angle_increment;
                    let local_x = range as f64 * (angle as f64).cos();
                    let local_y = range as f64 * (angle as f64).sin();
                    let world =
                        tf_iso.transform_point(&Point3::new(local_x, local_y, 0.0));
                    new_points.push((world.x, world.y));
                }

                *points_cb.write().unwrap() = new_points;
            },
        )?;

        Ok(LaserListener {
            config,
            points,
            _sub: Arc::new(sub),
        })
    }
}
