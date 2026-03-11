use crate::config::SendPoseConfig;
use crate::ros::ROS;
use rclrs::RclrsError;

pub trait BasePosePub: Send + Sync {
    fn get_topic(&self) -> &str;
    fn send(&self, x: f64, y: f64, yaw: f64, frame_id: &str);
}

fn pose_msg(x: f64, y: f64, yaw: f64) -> geometry_msgs::msg::Pose {
    let half_yaw = yaw * 0.5;
    let mut msg = geometry_msgs::msg::Pose::default();
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = 0.0;
    msg.orientation.z = half_yaw.sin();
    msg.orientation.w = half_yaw.cos();
    msg
}

pub struct PosePub {
    topic: String,
    publisher: rclrs::Publisher<geometry_msgs::msg::Pose>,
}

impl PosePub {
    pub fn new(ros: &ROS, topic: &str) -> Result<Self, RclrsError> {
        let publisher = ros.node.create_publisher::<geometry_msgs::msg::Pose>(topic)?;
        Ok(PosePub { topic: topic.to_string(), publisher })
    }
}

impl BasePosePub for PosePub {
    fn get_topic(&self) -> &str { &self.topic }
    fn send(&self, x: f64, y: f64, yaw: f64, _frame_id: &str) {
        let _ = self.publisher.publish(&pose_msg(x, y, yaw));
    }
}

pub struct PoseStampedPub {
    topic: String,
    publisher: rclrs::Publisher<geometry_msgs::msg::PoseStamped>,
}

impl PoseStampedPub {
    pub fn new(ros: &ROS, topic: &str) -> Result<Self, RclrsError> {
        let publisher = ros.node.create_publisher::<geometry_msgs::msg::PoseStamped>(topic)?;
        Ok(PoseStampedPub { topic: topic.to_string(), publisher })
    }
}

impl BasePosePub for PoseStampedPub {
    fn get_topic(&self) -> &str { &self.topic }
    fn send(&self, x: f64, y: f64, yaw: f64, frame_id: &str) {
        let mut msg = geometry_msgs::msg::PoseStamped::default();
        msg.header.frame_id = frame_id.to_string();
        msg.pose = pose_msg(x, y, yaw);
        let _ = self.publisher.publish(&msg);
    }
}

pub struct PoseCovPub {
    topic: String,
    publisher: rclrs::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>,
}

impl PoseCovPub {
    pub fn new(ros: &ROS, topic: &str) -> Result<Self, RclrsError> {
        let publisher = ros.node.create_publisher::<geometry_msgs::msg::PoseWithCovarianceStamped>(topic)?;
        Ok(PoseCovPub { topic: topic.to_string(), publisher })
    }
}

impl BasePosePub for PoseCovPub {
    fn get_topic(&self) -> &str { &self.topic }
    fn send(&self, x: f64, y: f64, yaw: f64, frame_id: &str) {
        let mut msg = geometry_msgs::msg::PoseWithCovarianceStamped::default();
        msg.header.frame_id = frame_id.to_string();
        msg.pose.pose = pose_msg(x, y, yaw);
        let _ = self.publisher.publish(&msg);
    }
}

pub struct CmdVelPub {
    publisher: rclrs::Publisher<geometry_msgs::msg::Twist>,
}

impl CmdVelPub {
    pub fn new(ros: &ROS, topic: &str) -> Result<Self, RclrsError> {
        let publisher = ros.node.create_publisher::<geometry_msgs::msg::Twist>(topic)?;
        Ok(CmdVelPub { publisher })
    }

    pub fn send(&self, vx: f64, vy: f64, wz: f64) {
        let mut msg = geometry_msgs::msg::Twist::default();
        msg.linear.x = vx;
        msg.linear.y = vy;
        msg.angular.z = wz;
        let _ = self.publisher.publish(&msg);
    }
}

/// Creates pose publishers from config, skipping any that fail with a warning.
pub fn create_pose_publishers(ros: &ROS, configs: &[SendPoseConfig]) -> Vec<Box<dyn BasePosePub>> {
    configs.iter().filter_map(|cfg| {
        let result: Result<Box<dyn BasePosePub>, RclrsError> = match cfg.msg_type.as_str() {
            "Pose" => PosePub::new(ros, &cfg.topic)
                .map(|p| Box::new(p) as Box<dyn BasePosePub>),
            "PoseStamped" => PoseStampedPub::new(ros, &cfg.topic)
                .map(|p| Box::new(p) as Box<dyn BasePosePub>),
            "PoseWithCovarianceStamped" => PoseCovPub::new(ros, &cfg.topic)
                .map(|p| Box::new(p) as Box<dyn BasePosePub>),
            other => {
                eprintln!("Unknown pose msg_type '{}' for topic '{}'", other, cfg.topic);
                return None;
            }
        };
        match result {
            Ok(p) => Some(p),
            Err(e) => {
                eprintln!("Failed to create publisher for '{}': {:?}", cfg.topic, e);
                None
            }
        }
    }).collect()
}
