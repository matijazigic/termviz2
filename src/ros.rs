use rclrs::{Context, CreateBasicExecutor, ExecutorCommands, Node, RclrsError, SpinOptions};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::task::JoinHandle;
use transforms::{
    geometry::{Quaternion, Transform, Vector3},
    time::Timestamp,
    Registry,
};

pub struct ROS {
    pub node: Node,
    pub tf: Shared<Registry>,
    tf_msg_count: Shared<u32>,
    /// child -> (parent, latest dynamic stamp). Static frames have None stamp.
    pub frames: Shared<HashMap<String, (String, Option<Timestamp>)>>,
    executor_commands: Arc<ExecutorCommands>,
    _spin_handle: Option<JoinHandle<()>>,
    // Keep subscriptions alive — dropping them unsubscribes.
    _subs: Vec<Arc<dyn std::any::Any + Send + Sync>>,
}

impl ROS {
    pub fn new(
        tf_topic: &str,
        tf_static_topic: &str,
    ) -> Result<Self, RclrsError> {
        let context = Context::default_from_env()?;
        let mut executor = context.create_basic_executor();
        let node = executor.create_node("termviz2")?;

        let tf: Shared<Registry> = shared(Registry::new(Duration::from_secs(10)));
        let tf_msg_count: Shared<u32> = shared(0);
        let frames: Shared<HashMap<String, (String, Option<Timestamp>)>> = shared(HashMap::new());

        let mut subs: Vec<Arc<dyn std::any::Any + Send + Sync>> = Vec::new();

        // dynamic transforms
        let tf_dyn = Arc::clone(&tf);
        let count_dyn = Arc::clone(&tf_msg_count);
        let frames_dyn = Arc::clone(&frames);
        let sub_tf = node.create_subscription::<tf2_msgs::msg::TFMessage, _>(
            tf_topic,
            move |msg: tf2_msgs::msg::TFMessage| {
                *count_dyn.lock().unwrap() += 1;
                let mut reg = tf_dyn.lock().unwrap();
                let mut frm = frames_dyn.lock().unwrap();
                for t in &msg.transforms {
                    let stamp: Timestamp = Timestamp {
                        t: t.header.stamp.sec as u128 * 1_000_000_000
                            + t.header.stamp.nanosec as u128,
                    };
                    let entry = frm
                        .entry(t.child_frame_id.clone())
                        .or_insert((t.header.frame_id.clone(), None));
                    entry.1 = Some(match entry.1 {
                        Some(prev) if prev > stamp => prev,
                        _ => stamp,
                    });
                    reg.add_transform(stamped_to_transform(t));
                }
            },
        )?;
        subs.push(Arc::new(sub_tf));

        // static transforms
        let tf_static = Arc::clone(&tf);
        let frames_static = Arc::clone(&frames);
        let sub_static = node.create_subscription::<tf2_msgs::msg::TFMessage, _>(
            tf_static_topic,
            move |msg: tf2_msgs::msg::TFMessage| {
                let mut reg = tf_static.lock().unwrap();
                let mut frm = frames_static.lock().unwrap();
                for t in &msg.transforms {
                    // Static frames: insert with None stamp (no dynamic stamp needed)
                    frm.entry(t.child_frame_id.clone())
                        .or_insert((t.header.frame_id.clone(), None));
                    reg.add_transform(stamped_to_transform(t));
                }
            },
        )?;
        subs.push(Arc::new(sub_static));

        let executor_commands = Arc::clone(executor.commands());
        let _spin_handle = Some(tokio::task::spawn_blocking(move || {
            executor.spin(SpinOptions::default());
        }));

        Ok(ROS { node, tf, tf_msg_count, frames, executor_commands, _spin_handle, _subs: subs })
    }

    /// Returns true once at least `n` dynamic TF messages have been received.
    pub fn has_received_n(&self, n: u32) -> bool {
        *self.tf_msg_count.lock().unwrap() >= n
    }

    /// Block until at least `n` dynamic TF messages have been received, printing status each second.
    pub fn wait_for_n_messages(&self, n: u32) {
        loop {
            self.print_tf_status();
            if self.has_received_n(n) {
                break;
            }
            std::thread::sleep(Duration::from_secs(1));
        }
    }

    /// Wait up to `timeout` for the transform from `parent` to `child` to become available.
    /// Queries at the minimum latest stamp across all dynamic frames — the "safe" time where
    /// every frame's buffer has at least one entry and interpolation succeeds.
    pub fn wait_for_transform(&self, parent: &str, child: &str, timeout: Duration) -> bool {
        let step = Duration::from_millis(100);
        let mut elapsed = Duration::ZERO;
        while elapsed < timeout {
            let safe_stamp = self.min_dynamic_stamp();
            if let Some(stamp) = safe_stamp {
                if self.tf.lock().unwrap().get_transform(parent, child, stamp).is_ok() {
                    return true;
                }
            }
            std::thread::sleep(step);
            elapsed += step;
        }
        false
    }

    /// Returns the minimum of all per-frame latest dynamic stamps.
    /// All dynamic frames are guaranteed to have an entry at or before this time.
    pub fn min_dynamic_stamp(&self) -> Option<Timestamp> {
        self.frames
            .lock()
            .unwrap()
            .values()
            .filter_map(|(_, stamp)| *stamp)
            .min()
    }

    pub fn print_tf_status(&self) {
        let count = *self.tf_msg_count.lock().unwrap();
        println!("TF messages received: {}", count);
        let frm = self.frames.lock().unwrap();
        if frm.is_empty() {
            println!("  No transforms received.");
        } else {
            println!("  Connections ({}):", frm.len());
            for (child, (parent, _)) in frm.iter() {
                println!("    {} -> {}", parent, child);
            }
        }
    }

    pub fn ok(&self) -> bool {
        self.executor_commands.context().ok()
    }
}

impl Drop for ROS {
    fn drop(&mut self) {
        self.executor_commands.halt_spinning();
    }
}

fn stamped_to_transform(t: &geometry_msgs::msg::TransformStamped) -> Transform {
    let tr = &t.transform;
    Transform {
        translation: Vector3::new(tr.translation.x, tr.translation.y, tr.translation.z),
        rotation: Quaternion { w: tr.rotation.w, x: tr.rotation.x, y: tr.rotation.y, z: tr.rotation.z },
        timestamp: Timestamp { t: t.header.stamp.sec as u128 * 1_000_000_000 + t.header.stamp.nanosec as u128 },
        parent: t.header.frame_id.clone().into(),
        child: t.child_frame_id.clone().into(),
    }
}

/// Convenience wrapper so subscribers can share data with the UI thread.
pub type Shared<T> = Arc<Mutex<T>>;

pub fn shared<T>(val: T) -> Shared<T> {
    Arc::new(Mutex::new(val))
}
