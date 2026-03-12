use crate::ros::ROS;
use rclrs::SubscriptionOptions;
use std::any::Any;
use std::sync::{Arc, RwLock};

/// Returns the available plottable field paths for a given ROS type string.
pub fn fields_for_type(type_str: &str) -> Option<&'static [&'static str]> {
    match type_str {
        "nav_msgs/msg/Odometry" => Some(&[
            "twist.twist.linear.x", "twist.twist.linear.y", "twist.twist.linear.z",
            "twist.twist.angular.x", "twist.twist.angular.y", "twist.twist.angular.z",
            "pose.pose.position.x", "pose.pose.position.y", "pose.pose.position.z",
        ]),
        "geometry_msgs/msg/Twist" => Some(&[
            "linear.x", "linear.y", "linear.z",
            "angular.x", "angular.y", "angular.z",
        ]),
        "geometry_msgs/msg/TwistStamped" => Some(&[
            "twist.linear.x", "twist.linear.y", "twist.linear.z",
            "twist.angular.x", "twist.angular.y", "twist.angular.z",
        ]),
        "geometry_msgs/msg/PoseStamped" => Some(&[
            "pose.position.x", "pose.position.y", "pose.position.z",
            "pose.orientation.x", "pose.orientation.y", "pose.orientation.z",
        ]),
        "geometry_msgs/msg/PoseWithCovarianceStamped" => Some(&[
            "pose.pose.position.x", "pose.pose.position.y", "pose.pose.position.z",
            "pose.pose.orientation.x", "pose.pose.orientation.y", "pose.pose.orientation.z",
        ]),
        "sensor_msgs/msg/Imu" => Some(&[
            "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
            "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
            "orientation.x", "orientation.y", "orientation.z", "orientation.w",
        ]),
        "sensor_msgs/msg/BatteryState" => Some(&[
            "voltage", "current", "percentage", "temperature", "charge", "capacity",
        ]),
        "sensor_msgs/msg/Range" => Some(&[
            "range",
        ]),
        "sensor_msgs/msg/NavSatFix" => Some(&[
            "latitude", "longitude", "altitude",
        ]),
        "sensor_msgs/msg/MagneticField" => Some(&[
            "magnetic_field.x", "magnetic_field.y", "magnetic_field.z",
        ]),
        "sensor_msgs/msg/Temperature" => Some(&[
            "temperature",
        ]),
        "sensor_msgs/msg/FluidPressure" => Some(&[
            "fluid_pressure",
        ]),
        _ => None,
    }
}

/// Maps a full ROS type string to the short key used internally.
pub fn short_msg_type(type_str: &str) -> Option<&'static str> {
    match type_str {
        "nav_msgs/msg/Odometry"                       => Some("Odometry"),
        "geometry_msgs/msg/Twist"                     => Some("Twist"),
        "geometry_msgs/msg/TwistStamped"              => Some("TwistStamped"),
        "geometry_msgs/msg/PoseStamped"               => Some("PoseStamped"),
        "geometry_msgs/msg/PoseWithCovarianceStamped" => Some("PoseWithCovarianceStamped"),
        "sensor_msgs/msg/Imu"                         => Some("Imu"),
        "sensor_msgs/msg/BatteryState"                => Some("BatteryState"),
        "sensor_msgs/msg/Range"                       => Some("Range"),
        "sensor_msgs/msg/NavSatFix"                   => Some("NavSatFix"),
        "sensor_msgs/msg/MagneticField"               => Some("MagneticField"),
        "sensor_msgs/msg/Temperature"                 => Some("Temperature"),
        "sensor_msgs/msg/FluidPressure"               => Some("FluidPressure"),
        _ => None,
    }
}

fn extract_odometry(msg: &nav_msgs::msg::Odometry, field: &str) -> Option<f64> {
    match field {
        "twist.twist.linear.x"  => Some(msg.twist.twist.linear.x),
        "twist.twist.linear.y"  => Some(msg.twist.twist.linear.y),
        "twist.twist.linear.z"  => Some(msg.twist.twist.linear.z),
        "twist.twist.angular.x" => Some(msg.twist.twist.angular.x),
        "twist.twist.angular.y" => Some(msg.twist.twist.angular.y),
        "twist.twist.angular.z" => Some(msg.twist.twist.angular.z),
        "pose.pose.position.x"  => Some(msg.pose.pose.position.x),
        "pose.pose.position.y"  => Some(msg.pose.pose.position.y),
        "pose.pose.position.z"  => Some(msg.pose.pose.position.z),
        _ => None,
    }
}

fn extract_twist(t: &geometry_msgs::msg::Twist, field: &str) -> Option<f64> {
    match field {
        "linear.x"  => Some(t.linear.x),
        "linear.y"  => Some(t.linear.y),
        "linear.z"  => Some(t.linear.z),
        "angular.x" => Some(t.angular.x),
        "angular.y" => Some(t.angular.y),
        "angular.z" => Some(t.angular.z),
        _ => None,
    }
}

fn extract_imu(msg: &sensor_msgs::msg::Imu, field: &str) -> Option<f64> {
    match field {
        "linear_acceleration.x" => Some(msg.linear_acceleration.x),
        "linear_acceleration.y" => Some(msg.linear_acceleration.y),
        "linear_acceleration.z" => Some(msg.linear_acceleration.z),
        "angular_velocity.x"    => Some(msg.angular_velocity.x),
        "angular_velocity.y"    => Some(msg.angular_velocity.y),
        "angular_velocity.z"    => Some(msg.angular_velocity.z),
        "orientation.x"         => Some(msg.orientation.x),
        "orientation.y"         => Some(msg.orientation.y),
        "orientation.z"         => Some(msg.orientation.z),
        "orientation.w"         => Some(msg.orientation.w),
        _ => None,
    }
}

fn extract_battery(msg: &sensor_msgs::msg::BatteryState, field: &str) -> Option<f64> {
    match field {
        "voltage"    => Some(msg.voltage as f64),
        "current"    => Some(msg.current as f64),
        "percentage" => Some(msg.percentage as f64),
        "temperature" => Some(msg.temperature as f64),
        "charge"     => Some(msg.charge as f64),
        "capacity"   => Some(msg.capacity as f64),
        _ => None,
    }
}

fn extract_pose(p: &geometry_msgs::msg::Pose, field: &str) -> Option<f64> {
    match field {
        "position.x"    => Some(p.position.x),
        "position.y"    => Some(p.position.y),
        "position.z"    => Some(p.position.z),
        "orientation.x" => Some(p.orientation.x),
        "orientation.y" => Some(p.orientation.y),
        "orientation.z" => Some(p.orientation.z),
        "orientation.w" => Some(p.orientation.w),
        _ => None,
    }
}

pub struct PlotListener {
    pub topic: String,
    pub fields: Vec<String>,
    pub values: Arc<RwLock<Vec<f64>>>,
    _sub: Arc<dyn Any + Send + Sync>,
}

impl PlotListener {
    pub fn new(topic: &str, msg_type: &str, fields: Vec<String>, ros: &Arc<ROS>) -> Option<Self> {
        let n = fields.len();
        let values: Arc<RwLock<Vec<f64>>> = Arc::new(RwLock::new(vec![0.0; n]));

        let sub: Arc<dyn Any + Send + Sync> = match msg_type {
            "Odometry" => {
                let vals = Arc::clone(&values);
                let flds = fields.clone();
                match ros.node.create_subscription::<nav_msgs::msg::Odometry, _>(
                    SubscriptionOptions::new(topic),
                    move |msg: nav_msgs::msg::Odometry| {
                        let mut v = vals.write().unwrap();
                        for (i, f) in flds.iter().enumerate() {
                            if let Some(val) = extract_odometry(&msg, f) { v[i] = val; }
                        }
                    },
                ) {
                    Ok(s) => Arc::new(s),
                    Err(e) => { eprintln!("PlotListener '{}': {:?}", topic, e); return None; }
                }
            }
            "Twist" => {
                let vals = Arc::clone(&values);
                let flds = fields.clone();
                match ros.node.create_subscription::<geometry_msgs::msg::Twist, _>(
                    SubscriptionOptions::new(topic),
                    move |msg: geometry_msgs::msg::Twist| {
                        let mut v = vals.write().unwrap();
                        for (i, f) in flds.iter().enumerate() {
                            if let Some(val) = extract_twist(&msg, f) { v[i] = val; }
                        }
                    },
                ) {
                    Ok(s) => Arc::new(s),
                    Err(e) => { eprintln!("PlotListener '{}': {:?}", topic, e); return None; }
                }
            }
            "TwistStamped" => {
                let vals = Arc::clone(&values);
                let flds = fields.clone();
                match ros.node.create_subscription::<geometry_msgs::msg::TwistStamped, _>(
                    SubscriptionOptions::new(topic),
                    move |msg: geometry_msgs::msg::TwistStamped| {
                        let mut v = vals.write().unwrap();
                        for (i, f) in flds.iter().enumerate() {
                            if let Some(val) = extract_twist(&msg.twist, f) { v[i] = val; }
                        }
                    },
                ) {
                    Ok(s) => Arc::new(s),
                    Err(e) => { eprintln!("PlotListener '{}': {:?}", topic, e); return None; }
                }
            }
            "PoseStamped" => {
                let vals = Arc::clone(&values);
                let flds = fields.clone();
                match ros.node.create_subscription::<geometry_msgs::msg::PoseStamped, _>(
                    SubscriptionOptions::new(topic),
                    move |msg: geometry_msgs::msg::PoseStamped| {
                        let mut v = vals.write().unwrap();
                        for (i, f) in flds.iter().enumerate() {
                            if let Some(val) = extract_pose(&msg.pose, f) { v[i] = val; }
                        }
                    },
                ) {
                    Ok(s) => Arc::new(s),
                    Err(e) => { eprintln!("PlotListener '{}': {:?}", topic, e); return None; }
                }
            }
            "PoseWithCovarianceStamped" => {
                let vals = Arc::clone(&values);
                let flds = fields.clone();
                match ros.node.create_subscription::<geometry_msgs::msg::PoseWithCovarianceStamped, _>(
                    SubscriptionOptions::new(topic),
                    move |msg: geometry_msgs::msg::PoseWithCovarianceStamped| {
                        let mut v = vals.write().unwrap();
                        for (i, f) in flds.iter().enumerate() {
                            if let Some(val) = extract_pose(&msg.pose.pose, f) { v[i] = val; }
                        }
                    },
                ) {
                    Ok(s) => Arc::new(s),
                    Err(e) => { eprintln!("PlotListener '{}': {:?}", topic, e); return None; }
                }
            }
            "Imu" => {
                let vals = Arc::clone(&values);
                let flds = fields.clone();
                match ros.node.create_subscription::<sensor_msgs::msg::Imu, _>(
                    SubscriptionOptions::new(topic),
                    move |msg: sensor_msgs::msg::Imu| {
                        let mut v = vals.write().unwrap();
                        for (i, f) in flds.iter().enumerate() {
                            if let Some(val) = extract_imu(&msg, f) { v[i] = val; }
                        }
                    },
                ) {
                    Ok(s) => Arc::new(s),
                    Err(e) => { eprintln!("PlotListener '{}': {:?}", topic, e); return None; }
                }
            }
            "BatteryState" => {
                let vals = Arc::clone(&values);
                let flds = fields.clone();
                match ros.node.create_subscription::<sensor_msgs::msg::BatteryState, _>(
                    SubscriptionOptions::new(topic),
                    move |msg: sensor_msgs::msg::BatteryState| {
                        let mut v = vals.write().unwrap();
                        for (i, f) in flds.iter().enumerate() {
                            if let Some(val) = extract_battery(&msg, f) { v[i] = val; }
                        }
                    },
                ) {
                    Ok(s) => Arc::new(s),
                    Err(e) => { eprintln!("PlotListener '{}': {:?}", topic, e); return None; }
                }
            }
            "Range" => {
                let vals = Arc::clone(&values);
                match ros.node.create_subscription::<sensor_msgs::msg::Range, _>(
                    SubscriptionOptions::new(topic),
                    move |msg: sensor_msgs::msg::Range| {
                        vals.write().unwrap()[0] = msg.range as f64;
                    },
                ) {
                    Ok(s) => Arc::new(s),
                    Err(e) => { eprintln!("PlotListener '{}': {:?}", topic, e); return None; }
                }
            }
            "NavSatFix" => {
                let vals = Arc::clone(&values);
                let flds = fields.clone();
                match ros.node.create_subscription::<sensor_msgs::msg::NavSatFix, _>(
                    SubscriptionOptions::new(topic),
                    move |msg: sensor_msgs::msg::NavSatFix| {
                        let mut v = vals.write().unwrap();
                        for (i, f) in flds.iter().enumerate() {
                            let val = match f.as_str() {
                                "latitude"  => msg.latitude,
                                "longitude" => msg.longitude,
                                "altitude"  => msg.altitude,
                                _ => continue,
                            };
                            v[i] = val;
                        }
                    },
                ) {
                    Ok(s) => Arc::new(s),
                    Err(e) => { eprintln!("PlotListener '{}': {:?}", topic, e); return None; }
                }
            }
            "MagneticField" => {
                let vals = Arc::clone(&values);
                let flds = fields.clone();
                match ros.node.create_subscription::<sensor_msgs::msg::MagneticField, _>(
                    SubscriptionOptions::new(topic),
                    move |msg: sensor_msgs::msg::MagneticField| {
                        let mut v = vals.write().unwrap();
                        for (i, f) in flds.iter().enumerate() {
                            let val = match f.as_str() {
                                "magnetic_field.x" => msg.magnetic_field.x,
                                "magnetic_field.y" => msg.magnetic_field.y,
                                "magnetic_field.z" => msg.magnetic_field.z,
                                _ => continue,
                            };
                            v[i] = val;
                        }
                    },
                ) {
                    Ok(s) => Arc::new(s),
                    Err(e) => { eprintln!("PlotListener '{}': {:?}", topic, e); return None; }
                }
            }
            "Temperature" => {
                let vals = Arc::clone(&values);
                match ros.node.create_subscription::<sensor_msgs::msg::Temperature, _>(
                    SubscriptionOptions::new(topic),
                    move |msg: sensor_msgs::msg::Temperature| {
                        vals.write().unwrap()[0] = msg.temperature;
                    },
                ) {
                    Ok(s) => Arc::new(s),
                    Err(e) => { eprintln!("PlotListener '{}': {:?}", topic, e); return None; }
                }
            }
            "FluidPressure" => {
                let vals = Arc::clone(&values);
                match ros.node.create_subscription::<sensor_msgs::msg::FluidPressure, _>(
                    SubscriptionOptions::new(topic),
                    move |msg: sensor_msgs::msg::FluidPressure| {
                        vals.write().unwrap()[0] = msg.fluid_pressure;
                    },
                ) {
                    Ok(s) => Arc::new(s),
                    Err(e) => { eprintln!("PlotListener '{}': {:?}", topic, e); return None; }
                }
            }
            other => {
                eprintln!("PlotListener: unsupported msg_type '{}' for '{}'", other, topic);
                return None;
            }
        };

        Some(PlotListener { topic: topic.to_string(), fields, values, _sub: sub })
    }
}
