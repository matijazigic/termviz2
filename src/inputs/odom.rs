use crate::ros::ROS;
use rclrs::{RclrsError, SubscriptionOptions};
use std::sync::{Arc, RwLock};

/// Latest odometry velocity: (vx, vy, wz)
pub type OdomVel = (f64, f64, f64);

pub struct OdomListener {
    pub vel: Arc<RwLock<OdomVel>>,
    _sub: Arc<dyn std::any::Any + Send + Sync>,
}

impl OdomListener {
    pub fn new(topic: String, ros: &Arc<ROS>) -> Result<Self, RclrsError> {
        let vel: Arc<RwLock<OdomVel>> = Arc::new(RwLock::new((0.0, 0.0, 0.0)));
        let vel_cb = Arc::clone(&vel);

        let sub = ros.node.create_subscription::<nav_msgs::msg::Odometry, _>(
            SubscriptionOptions::new(&topic),
            move |msg: nav_msgs::msg::Odometry| {
                let vx = msg.twist.twist.linear.x;
                let vy = msg.twist.twist.linear.y;
                let wz = msg.twist.twist.angular.z;
                *vel_cb.write().unwrap() = (vx, vy, wz);
            },
        )?;

        Ok(OdomListener { vel, _sub: Arc::new(sub) })
    }
}
