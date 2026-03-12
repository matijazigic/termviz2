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
        let default_color = Color::Rgb(config.default_color.r, config.default_color.g, config.default_color.b);

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
                let step = msg.point_step as u32;

                // Read and transform all points.
                // world_z is kept separately for z-gradient colorization.
                let mut points: Vec<ColoredPoint> = Vec::with_capacity(n_pts);
                let mut world_z: Vec<f64> = Vec::with_capacity(n_pts);
                let mut min_z = f64::MAX;
                let mut max_z = f64::MIN;

                for i in 0..n_pts {
                    let base = i as u32 * step;
                    let lx = read_f32(&msg.data, base + x_off) as f64;
                    let ly = read_f32(&msg.data, base + y_off) as f64;
                    let lz = read_f32(&msg.data, base + z_off) as f64;
                    let w = tf_iso.transform_point(&Point3::new(lx, ly, lz));
                    if w.z < min_z { min_z = w.z; }
                    if w.z > max_z { max_z = w.z; }
                    world_z.push(w.z);
                    points.push(ColoredPoint { x: w.x, y: w.y, color: Color::Reset });
                }

                // Colorize from per-point RGB field or z-gradient.
                if use_rgb {
                    if let Some(rgb_off) = field_offset("rgb", &msg.fields) {
                        for (i, pt) in points.iter_mut().enumerate() {
                            let idx: u32 = i as u32 * step + rgb_off;
                            pt.color = Color::Rgb(
                                msg.data[(idx + 2) as usize],
                                msg.data[(idx + 1) as usize],
                                msg.data[idx as usize],
                            );
                        }
                    } else {
                        for pt in points.iter_mut() {
                            pt.color = default_color;
                        }
                    }
                } else {
                    let grad = colorgrad::preset::turbo();
                    let z_range = (max_z - min_z).max(f64::EPSILON) as f32;
                    for (pt, &wz) in points.iter_mut().zip(world_z.iter()) {
                        let t = ((wz - min_z) as f32 / z_range).clamp(0.0, 1.0);
                        let c = grad.at(t).to_rgba8();
                        pt.color = Color::Rgb(c[0], c[1], c[2]);
                    }
                }

                // Filter NaN points.
                points.retain(|pt| pt.x.is_finite() && pt.y.is_finite());

                *points_cb.write().unwrap() = points;
            },
        )?;

        Ok(PointCloud2Listener { points, _sub: Arc::new(sub) })
    }
}
