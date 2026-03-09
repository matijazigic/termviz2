use crate::config::PointCloud2ListenerConfig;
use crate::ros::ROS;
use byteorder::{ByteOrder, LittleEndian};
use colorgrad::Gradient;
use nalgebra::geometry::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};
use ratatui::style::Color;
use rclrs::{RclrsError, SubscriptionOptions};
use std::sync::{Arc, RwLock};

#[derive(Clone)]
pub struct ColoredPoint {
    pub x: f64,
    pub y: f64,
    pub color: Color,
}

pub struct PointCloud2Listener {
    pub config: PointCloud2ListenerConfig,
    pub points: Arc<RwLock<Vec<ColoredPoint>>>,
    _sub: Arc<dyn std::any::Any + Send + Sync>,
}

fn field_offset(name: &str, fields: &[sensor_msgs::msg::PointField]) -> Option<u32> {
    fields.iter().find(|f| f.name == name).map(|f| f.offset)
}

fn read_f32(data: &[u8], idx: u32) -> f32 {
    LittleEndian::read_f32(&data[idx as usize..(idx + 4) as usize])
}

impl PointCloud2Listener {
    pub fn new(
        config: PointCloud2ListenerConfig,
        ros: Arc<ROS>,
        fixed_frame: String,
    ) -> Result<Self, RclrsError> {
        let points: Arc<RwLock<Vec<ColoredPoint>>> = Arc::new(RwLock::new(Vec::new()));
        let points_cb = Arc::clone(&points);
        let tf = Arc::clone(&ros.tf);
        let frames = Arc::clone(&ros.frames);
        let use_rgb = config.use_rgb;

        let sub = ros.node.create_subscription::<sensor_msgs::msg::PointCloud2, _>(
            SubscriptionOptions::new(&config.topic),
            move |msg: sensor_msgs::msg::PointCloud2| {
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
                    let tra: nalgebra::Translation<f64, 3> = Translation3::new(t.translation.x, t.translation.y, t.translation.z);
                    let rot = UnitQuaternion::new_normalize(Quaternion::new(
                        t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z,
                    ));
                    Isometry3::from_parts(tra, rot)
                };

                let Some(x_off) = field_offset("x", &msg.fields) else { return };
                let Some(y_off) = field_offset("y", &msg.fields) else { return };
                let Some(z_off) = field_offset("z", &msg.fields) else { return };

                let n_pts = (msg.width * msg.height) as usize;
                let step = msg.point_step as usize;

                // First pass: transform points and compute z range for colorization.
                let mut raw: Vec<(f64, f64, f64)> = Vec::with_capacity(n_pts);
                let mut min_z = f64::MAX;
                let mut max_z = f64::MIN;

                for i in 0..n_pts {
                    let base = (i * step) as u32;
                    let lx = read_f32(&msg.data, base + x_off) as f64;
                    let ly = read_f32(&msg.data, base + y_off) as f64;
                    let lz = read_f32(&msg.data, base + z_off) as f64;
                    if lx.is_nan() || ly.is_nan() || lz.is_nan() {
                        continue;
                    }
                    let w = tf_iso.transform_point(&Point3::new(lx, ly, lz));
                    if w.z < min_z { min_z = w.z; }
                    if w.z > max_z { max_z = w.z; }
                    raw.push((w.x, w.y, w.z));
                }

                let rgb_off = if use_rgb { field_offset("rgb", &msg.fields) } else { None };
                let z_range = (max_z - min_z).max(f64::EPSILON);
                let grad = colorgrad::preset::turbo();

                // Second pass: assign colors.
                let new_points: Vec<ColoredPoint> = raw
                    .iter()
                    .enumerate()
                    .map(|(i, &(x, y, z))| {
                        let color = if let Some(off) = rgb_off {
                            let idx = (i * step) as u32 + off;
                            Color::Rgb(
                                msg.data[(idx + 2) as usize],
                                msg.data[(idx + 1) as usize],
                                msg.data[idx as usize],
                            )
                        } else {
                            let t = ((z - min_z) / z_range) as f32;
                            let c = grad.at(t).to_rgba8();
                            Color::Rgb(c[0], c[1], c[2])
                        };
                        ColoredPoint { x, y, color }
                    })
                    .collect();

                *points_cb.write().unwrap() = new_points;
            },
        )?;

        Ok(PointCloud2Listener { config, points, _sub: Arc::new(sub) })
    }
}
