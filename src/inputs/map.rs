use crate::ros::ROS;

use crate::config::MapListenerConfig;
use nalgebra::geometry::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};
use ratatui::style::Color;
use rclrs::{QoSDurabilityPolicy, RclrsError, SubscriptionOptions};
use std::sync::{Arc, RwLock};

/// One colour bucket: a ratatui colour and the world-space (x,y) points for it.
pub type ColorLayer = (Color, Vec<(f64, f64)>);

pub struct MapListener {
    pub config: MapListenerConfig,
    /// Pre-computed, colour-bucketed world-space points ready for rendering.
    /// Static map  → one layer using config.color.
    /// Costmap     → multiple layers using the RViz cost-colour gradient.
    pub layers: Arc<RwLock<Vec<ColorLayer>>>,
    _sub: Arc<dyn std::any::Any + Send + Sync>,
}

// ---------------------------------------------------------------------------
// RViz-style costmap colour mapping
// Nav2 publishes costmap values as i8 occupancy:
//   -1  = unknown / no information → dark grey
//    0  = free                      → not drawn
//   1-N = cost                      → green → yellow → orange → red
//   99+ = lethal (inscribed/lethal) → bright magenta
// ---------------------------------------------------------------------------

impl MapListener {
    pub fn new(
        config: MapListenerConfig,
        ros: Arc<ROS>,
        fixed_frame: String,
    ) -> Result<Self, RclrsError> {
        let layers: Arc<RwLock<Vec<ColorLayer>>> = Arc::new(RwLock::new(Vec::new()));
        let layers_cb = Arc::clone(&layers);

        let tf = Arc::clone(&ros.tf);
        let frames = Arc::clone(&ros.frames);

        let threshold = config.threshold;
        let transient_local = config.transient_local;
        let cfg_color = Color::Rgb(config.color.r, config.color.g, config.color.b);

        let mut sub_options = SubscriptionOptions::new(&config.topic);
        sub_options.qos.durability = if transient_local {
            QoSDurabilityPolicy::TransientLocal
        } else {
            QoSDurabilityPolicy::Volatile
        };

        let sub = ros.node.create_subscription::<nav_msgs::msg::OccupancyGrid, _>(
            sub_options,
            move |msg: nav_msgs::msg::OccupancyGrid| {
                let stamp = {
                    let frm = frames.lock().unwrap();
                    frm.values().filter_map(|(_, s)| *s).min()
                };
                let Some(stamp) = stamp else { return };

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
                let combined = tf_iso * Isometry3::from_parts(o_tra, o_rot);

                let width = msg.info.width as usize;
                let resolution = msg.info.resolution as f64;

                let new_layers = if transient_local {
                    // Static map: single colour layer, threshold filter.
                    let mut pts = Vec::new();
                    for (i, &cell) in msg.data.iter().enumerate() {
                        if cell < threshold { continue; }
                        let row = i / width;
                        let col = i % width;
                        let w = combined.transform_point(&Point3::new(
                            col as f64 * resolution,
                            row as f64 * resolution,
                            0.0,
                        ));
                        pts.push((w.x, w.y));
                    }
                    vec![(cfg_color, pts)]
                } else {
                    // Costmap: outline only.
                    // A cell is drawn only if it has cost > 0 and < 100 (not lethal)
                    // AND at least one 4-neighbour is free (cost == 0).
                    // This gives the boundary of the inflation zone, not the fill.
                    let height = msg.info.height as usize;
                    let data = &msg.data;
                    let mut outline = Vec::new();
                    for (i, &cell) in data.iter().enumerate() {
                        if cell <= 0 || cell >= 100 { continue; }
                        let row = i / width;
                        let col = i % width;
                        let has_free_neighbour =
                            (row > 0          && data[(row - 1) * width + col] == 0) ||
                            (row + 1 < height && data[(row + 1) * width + col] == 0) ||
                            (col > 0          && data[row * width + (col - 1)] == 0) ||
                            (col + 1 < width  && data[row * width + (col + 1)] == 0);
                        if !has_free_neighbour { continue; }
                        let w = combined.transform_point(&Point3::new(
                            col as f64 * resolution,
                            row as f64 * resolution,
                            0.0,
                        ));
                        outline.push((w.x, w.y));
                    }
                    vec![(cfg_color, outline)]
                };

                *layers_cb.write().unwrap() = new_layers;
            },
        )?;

        Ok(MapListener {
            config,
            layers,
            _sub: Arc::new(sub),
        })
    }
}
