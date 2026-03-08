use crate::ros::ROS;

use crate::config::MapListenerConfig;
use nalgebra::geometry::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};
use rclrs::{QoSProfile, QoSDurabilityPolicy, RclrsError, SubscriptionOptions};
use std::sync::{Arc, RwLock};

pub struct MapListener {
    pub config: MapListenerConfig,
    /// Pre-computed world-space (x, y) points ready for rendering.
    pub points: Arc<RwLock<Vec<(f64, f64)>>>,
    _sub: Arc<dyn std::any::Any + Send + Sync>,
}

impl MapListener {
    pub fn new(
        config: MapListenerConfig,
        ros: Arc<ROS>,
        fixed_frame: String,
    ) -> Result<Self, RclrsError> {
        let points: Arc<RwLock<Vec<(f64, f64)>>> = Arc::new(RwLock::new(Vec::new()));
        let points_cb = Arc::clone(&points);
        let threshold = config.threshold;

        // Clone what the closure needs so `ros` isn't borrowed by `create_subscription`.
        let tf = Arc::clone(&ros.tf);
        let frames = Arc::clone(&ros.frames);

        let mut sub_options = SubscriptionOptions::new(&config.topic);
        sub_options.qos.durability = QoSDurabilityPolicy::TransientLocal;
        let sub = ros.node.create_subscription::<nav_msgs::msg::OccupancyGrid, _>(
            sub_options,
            move |msg: nav_msgs::msg::OccupancyGrid| {
                // Get safe TF query timestamp.
                let stamp = {
                    let frm = frames.lock().unwrap();
                    frm.values().filter_map(|(_, s)| *s).min()
                };
                let Some(stamp) = stamp else { return };

                // Build the TF isometry from fixed_frame → map.header.frame_id.
                // If both are the same frame, use identity.
                let tf_iso: Isometry3<f64> = if msg.header.frame_id == fixed_frame {
                    Isometry3::identity()
                } else {
                    let result = tf.lock().unwrap()
                        .get_transform(&fixed_frame, &msg.header.frame_id, stamp);
                    let Ok(t) = result else { return };
                    let tra = Translation3::new(t.translation.x, t.translation.y, t.translation.z);
                    let rot = UnitQuaternion::new_normalize(Quaternion::new(
                        t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z,
                    ));
                    Isometry3::from_parts(tra, rot)
                };

                // Map origin isometry: converts grid indices → map.header.frame_id coords.
                let origin = &msg.info.origin;
                let o_tra = Translation3::new(
                    origin.position.x, origin.position.y, origin.position.z,
                );
                let o_rot = UnitQuaternion::new_normalize(Quaternion::new(
                    origin.orientation.w,
                    origin.orientation.x,
                    origin.orientation.y,
                    origin.orientation.z,
                ));
                let origin_iso = Isometry3::from_parts(o_tra, o_rot);

                // Combine both: world = tf_iso * origin_iso * local
                let combined = tf_iso * origin_iso;

                let width = msg.info.width as usize;
                let resolution = msg.info.resolution as f64;

                let mut new_points = Vec::new();
                for (i, &cell) in msg.data.iter().enumerate() {
                    if cell < threshold {
                        continue;
                    }
                    let row = i / width;
                    let col = i % width;
                    let world = combined.transform_point(&Point3::new(
                        col as f64 * resolution,
                        row as f64 * resolution,
                        0.0,
                    ));
                    new_points.push((world.x, world.y));
                }

                *points_cb.write().unwrap() = new_points;
            },
        )?;

        Ok(MapListener {
            config,
            points,
            _sub: Arc::new(sub),
        })
    }
}
