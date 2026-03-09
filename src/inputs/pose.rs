use crate::config::PoseListenerConfig;
use crate::ros::ROS;
use nalgebra::geometry::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use rclrs::{RclrsError, SubscriptionOptions};
use std::sync::{Arc, RwLock};

/// World-space pose: (x, y, yaw_radians)
pub type Pose2D = (f64, f64, f64);

fn make_iso(p: &geometry_msgs::msg::Point, q: &geometry_msgs::msg::Quaternion) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(p.x, p.y, p.z),
        UnitQuaternion::new_normalize(Quaternion::new(q.w, q.x, q.y, q.z)),
    )
}

fn iso_to_pose2d(iso: Isometry3<f64>) -> Pose2D {
    let (_, _, yaw) = iso.rotation.euler_angles();
    (iso.translation.x, iso.translation.y, yaw)
}

// ---------------------------------------------------------------------------

pub struct PoseStampedListener {
    pub config: PoseListenerConfig,
    pub pose: Arc<RwLock<Option<Pose2D>>>,
    _sub: Arc<dyn std::any::Any + Send + Sync>,
}

impl PoseStampedListener {
    pub fn new(
        config: PoseListenerConfig,
        ros: Arc<ROS>,
        fixed_frame: String,
    ) -> Result<Self, RclrsError> {
        let pose: Arc<RwLock<Option<Pose2D>>> = Arc::new(RwLock::new(None));
        let pose_cb = Arc::clone(&pose);
        let tf = Arc::clone(&ros.tf);
        let frames = Arc::clone(&ros.frames);

        let sub = ros.node.create_subscription::<geometry_msgs::msg::PoseStamped, _>(
            SubscriptionOptions::new(&config.topic),
            move |msg: geometry_msgs::msg::PoseStamped| {
                let tf_iso: Isometry3<f64> = if msg.header.frame_id == fixed_frame {
                    Isometry3::identity()
                } else {
                    let stamp = {
                        let frm = frames.lock().unwrap();
                        frm.values().filter_map(|(_, s)| *s).min()
                    };
                    let Some(stamp) = stamp else { return };
                    let result = tf
                        .lock()
                        .unwrap()
                        .get_transform(&fixed_frame, &msg.header.frame_id, stamp);
                    let Ok(t) = result else { return };
                    let tra = Translation3::new(t.translation.x, t.translation.y, t.translation.z);
                    let rot = UnitQuaternion::new_normalize(Quaternion::new(
                        t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z,
                    ));
                    Isometry3::from_parts(tra, rot)
                };

                let world_iso = tf_iso * make_iso(&msg.pose.position, &msg.pose.orientation);
                *pose_cb.write().unwrap() = Some(iso_to_pose2d(world_iso));
            },
        )?;

        Ok(PoseStampedListener { config, pose, _sub: Arc::new(sub) })
    }
}

// ---------------------------------------------------------------------------

pub struct PoseArrayListener {
    pub config: PoseListenerConfig,
    pub poses: Arc<RwLock<Vec<Pose2D>>>,
    _sub: Arc<dyn std::any::Any + Send + Sync>,
}

impl PoseArrayListener {
    pub fn new(
        config: PoseListenerConfig,
        ros: Arc<ROS>,
        fixed_frame: String,
    ) -> Result<Self, RclrsError> {
        let poses: Arc<RwLock<Vec<Pose2D>>> = Arc::new(RwLock::new(Vec::new()));
        let poses_cb = Arc::clone(&poses);
        let tf = Arc::clone(&ros.tf);
        let frames = Arc::clone(&ros.frames);

        let sub = ros.node.create_subscription::<geometry_msgs::msg::PoseArray, _>(
            SubscriptionOptions::new(&config.topic),
            move |msg: geometry_msgs::msg::PoseArray| {
                let tf_iso: Isometry3<f64> = if msg.header.frame_id == fixed_frame {
                    Isometry3::identity()
                } else {
                    let stamp = {
                        let frm = frames.lock().unwrap();
                        frm.values().filter_map(|(_, s)| *s).min()
                    };
                    let Some(stamp) = stamp else { return };
                    let result = tf
                        .lock()
                        .unwrap()
                        .get_transform(&fixed_frame, &msg.header.frame_id, stamp);
                    let Ok(t) = result else { return };
                    let tra = Translation3::new(t.translation.x, t.translation.y, t.translation.z);
                    let rot = UnitQuaternion::new_normalize(Quaternion::new(
                        t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z,
                    ));
                    Isometry3::from_parts(tra, rot)
                };

                let new_poses = msg.poses.iter()
                    .map(|p| iso_to_pose2d(tf_iso * make_iso(&p.position, &p.orientation)))
                    .collect();
                *poses_cb.write().unwrap() = new_poses;
            },
        )?;

        Ok(PoseArrayListener { config, poses, _sub: Arc::new(sub) })
    }
}
