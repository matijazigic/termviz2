use crate::config::ListenerConfig;
use crate::ros::ROS;
use nalgebra::base::Vector3;
use nalgebra::geometry::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};
use rclrs::{RclrsError, SubscriptionOptions};
use ratatui::style::Color;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

// ── line primitive ────────────────────────────────────────────────────────────

#[derive(Clone, Copy)]
pub struct MarkerLine {
    pub x1: f64,
    pub y1: f64,
    pub x2: f64,
    pub y2: f64,
    pub color: Color,
}

// ── marker store with lifecycle management ────────────────────────────────────

struct StoredMarker {
    lines: Vec<MarkerLine>,
    deadline: Option<Instant>,
}

pub struct MarkerStore {
    markers: HashMap<String, HashMap<i32, StoredMarker>>,
}

impl MarkerStore {
    pub fn new() -> Self {
        Self { markers: HashMap::new() }
    }

    pub fn add(
        &mut self,
        ns: String,
        id: i32,
        lines: Vec<MarkerLine>,
        lifetime_sec: i32,
        lifetime_nsec: u32,
    ) {
        let deadline = if lifetime_sec == 0 && lifetime_nsec == 0 {
            None
        } else {
            let dur = Duration::new(lifetime_sec.max(0) as u64, lifetime_nsec);
            Some(Instant::now() + dur)
        };
        self.markers
            .entry(ns)
            .or_default()
            .insert(id, StoredMarker { lines, deadline });
    }

    pub fn delete(&mut self, ns: &str, id: i32) {
        if let Some(ns_map) = self.markers.get_mut(ns) {
            ns_map.remove(&id);
        }
    }

    pub fn clear(&mut self) {
        self.markers.clear();
    }

    /// Returns all active lines, pruning expired markers in-place.
    pub fn get_lines(&mut self) -> Vec<MarkerLine> {
        let now = Instant::now();
        let mut result = Vec::new();
        for ns_map in self.markers.values_mut() {
            ns_map.retain(|_, m| m.deadline.map_or(true, |d| d > now));
            for m in ns_map.values() {
                result.extend_from_slice(&m.lines);
            }
        }
        result
    }
}

// ── geometry helpers ──────────────────────────────────────────────────────────

fn iso_from_pose(pose: &geometry_msgs::msg::Pose) -> Isometry3<f64> {
    let tra = Translation3::new(pose.position.x, pose.position.y, pose.position.z);
    let rot = UnitQuaternion::new_normalize(Quaternion::new(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
    ));
    Isometry3::from_parts(tra, rot)
}

fn tf_iso_from_transform(t: &transforms::geometry::Transform) -> Isometry3<f64> {
    let tra = Translation3::new(t.translation.x, t.translation.y, t.translation.z);
    let rot = UnitQuaternion::new_normalize(Quaternion::new(
        t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z,
    ));
    Isometry3::from_parts(tra, rot)
}

fn strips_to_lines(strips: &[Vec<Point3<f64>>], color: Color) -> Vec<MarkerLine> {
    let mut lines = Vec::new();
    for strip in strips {
        for w in strip.windows(2) {
            lines.push(MarkerLine { x1: w[0].x, y1: w[0].y, x2: w[1].x, y2: w[1].y, color });
        }
    }
    lines
}

// ── marker type parsers ───────────────────────────────────────────────────────

fn parse_cube(
    dim: &geometry_msgs::msg::Vector3,
    offset: &geometry_msgs::msg::Point,
    color: Color,
    iso: &Isometry3<f64>,
) -> Vec<MarkerLine> {
    let (roll, pitch, _) = iso.rotation.euler_angles();
    let mut strips: Vec<Vec<Point3<f64>>> = Vec::new();

    let top = vec![
        iso.transform_point(&Point3::new(offset.x + dim.x / 2.0, offset.y + dim.y / 2.0, offset.z + dim.z / 2.0)),
        iso.transform_point(&Point3::new(offset.x + dim.x / 2.0, offset.y - dim.y / 2.0, offset.z + dim.z / 2.0)),
        iso.transform_point(&Point3::new(offset.x - dim.x / 2.0, offset.y - dim.y / 2.0, offset.z + dim.z / 2.0)),
        iso.transform_point(&Point3::new(offset.x - dim.x / 2.0, offset.y + dim.y / 2.0, offset.z + dim.z / 2.0)),
        iso.transform_point(&Point3::new(offset.x + dim.x / 2.0, offset.y + dim.y / 2.0, offset.z + dim.z / 2.0)),
    ];
    strips.push(top);

    if roll.abs() > 0.0001 || pitch.abs() > 0.0001 {
        let bot = vec![
            iso.transform_point(&Point3::new(offset.x + dim.x / 2.0, offset.y + dim.y / 2.0, offset.z - dim.z / 2.0)),
            iso.transform_point(&Point3::new(offset.x + dim.x / 2.0, offset.y - dim.y / 2.0, offset.z - dim.z / 2.0)),
            iso.transform_point(&Point3::new(offset.x - dim.x / 2.0, offset.y - dim.y / 2.0, offset.z - dim.z / 2.0)),
            iso.transform_point(&Point3::new(offset.x - dim.x / 2.0, offset.y + dim.y / 2.0, offset.z - dim.z / 2.0)),
            iso.transform_point(&Point3::new(offset.x + dim.x / 2.0, offset.y + dim.y / 2.0, offset.z - dim.z / 2.0)),
        ];
        strips.push(bot);
        // vertical edges
        for (sx, sy) in [(1.0_f64, 1.0_f64), (-1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)] {
            strips.push(vec![
                iso.transform_point(&Point3::new(offset.x + sx * dim.x / 2.0, offset.y + sy * dim.y / 2.0, offset.z + dim.z / 2.0)),
                iso.transform_point(&Point3::new(offset.x + sx * dim.x / 2.0, offset.y + sy * dim.y / 2.0, offset.z - dim.z / 2.0)),
            ]);
        }
    }

    strips_to_lines(&strips, color)
}

fn parse_arrow(
    msg: &visualization_msgs::msg::Marker,
    color: Color,
    iso: &Isometry3<f64>,
) -> Vec<MarkerLine> {
    match msg.points.len() {
        0 => {
            // Method 1: pose + scale defines the arrow
            let p1 = iso.transform_point(&Point3::new(0.0, 0.0, 0.0));
            let p2 = iso.transform_point(&Point3::new(msg.scale.x, 0.0, 0.0));
            let angle = PI / 4.0;
            let r = msg.scale.y / 2.0 / angle.cos();
            let a = PI - angle;
            let b = PI + angle;
            let p3r = iso.transform_point(&Point3::new(msg.scale.x + r * a.cos(), r * a.sin(), 0.0));
            let p3l = iso.transform_point(&Point3::new(msg.scale.x + r * b.cos(), r * b.sin(), 0.0));
            vec![
                MarkerLine { x1: p1.x, y1: p1.y, x2: p2.x, y2: p2.y, color },
                MarkerLine { x1: p2.x, y1: p2.y, x2: p3r.x, y2: p3r.y, color },
                MarkerLine { x1: p2.x, y1: p2.y, x2: p3l.x, y2: p3l.y, color },
            ]
        }
        2 => {
            // Method 2: start/end points
            let start = &msg.points[0];
            let end = &msg.points[1];
            let p1 = iso.transform_point(&Point3::new(start.x, start.y, start.z));
            let p2 = iso.transform_point(&Point3::new(end.x, end.y, end.z));
            let head_trafo = Isometry3::face_towards(&p2, &p1, &Vector3::y());
            let half_w = msg.scale.y / 2.0;
            let r = (half_w.powi(2) + msg.scale.z.powi(2)).sqrt();
            let ang = half_w.atan2(msg.scale.z);
            let p3r = head_trafo.transform_point(&Point3::new(0.0, r * ang.sin(), r * ang.cos()));
            let p3l = head_trafo.transform_point(&Point3::new(0.0, -r * ang.sin(), r * ang.cos()));
            vec![
                MarkerLine { x1: p1.x, y1: p1.y, x2: p2.x, y2: p2.y, color },
                MarkerLine { x1: p2.x, y1: p2.y, x2: p3r.x, y2: p3r.y, color },
                MarkerLine { x1: p2.x, y1: p2.y, x2: p3l.x, y2: p3l.y, color },
            ]
        }
        _ => Vec::new(),
    }
}

fn parse_sphere(
    msg: &visualization_msgs::msg::Marker,
    color: Color,
    iso: &Isometry3<f64>,
) -> Vec<MarkerLine> {
    let sx = msg.scale.x;
    let sy = msg.scale.y;
    let sz = msg.scale.z;
    let mut lines = Vec::new();

    // cross through center
    let p = [
        iso.transform_point(&Point3::new(-sx * 0.5, 0.0, 0.0)),
        iso.transform_point(&Point3::new( sx * 0.5, 0.0, 0.0)),
        iso.transform_point(&Point3::new(0.0, -sy * 0.5, 0.0)),
        iso.transform_point(&Point3::new(0.0,  sy * 0.5, 0.0)),
        iso.transform_point(&Point3::new(0.0, 0.0, -sz * 0.5)),
        iso.transform_point(&Point3::new(0.0, 0.0,  sz * 0.5)),
    ];
    lines.push(MarkerLine { x1: p[0].x, y1: p[0].y, x2: p[1].x, y2: p[1].y, color });
    lines.push(MarkerLine { x1: p[2].x, y1: p[2].y, x2: p[3].x, y2: p[3].y, color });
    lines.push(MarkerLine { x1: p[4].x, y1: p[4].y, x2: p[5].x, y2: p[5].y, color });

    let n = 20usize;
    let step = 2.0 * PI / n as f64;
    for i in 0..n {
        let ifl = i as f64;
        // XY ellipse
        let pa = iso.transform_point(&Point3::new(0.5 * sx * (ifl * step).sin(), 0.5 * sy * (ifl * step).cos(), 0.0));
        let pb = iso.transform_point(&Point3::new(0.5 * sx * ((ifl + 1.0) * step).sin(), 0.5 * sy * ((ifl + 1.0) * step).cos(), 0.0));
        lines.push(MarkerLine { x1: pa.x, y1: pa.y, x2: pb.x, y2: pb.y, color });
        // XZ ellipse
        let pa = iso.transform_point(&Point3::new(0.5 * sx * (ifl * step).sin(), 0.0, 0.5 * sz * (ifl * step).cos()));
        let pb = iso.transform_point(&Point3::new(0.5 * sx * ((ifl + 1.0) * step).sin(), 0.0, 0.5 * sz * ((ifl + 1.0) * step).cos()));
        lines.push(MarkerLine { x1: pa.x, y1: pa.y, x2: pb.x, y2: pb.y, color });
        // YZ ellipse
        let pa = iso.transform_point(&Point3::new(0.0, 0.5 * sy * (ifl * step).cos(), 0.5 * sz * (ifl * step).sin()));
        let pb = iso.transform_point(&Point3::new(0.0, 0.5 * sy * ((ifl + 1.0) * step).cos(), 0.5 * sz * ((ifl + 1.0) * step).sin()));
        lines.push(MarkerLine { x1: pa.x, y1: pa.y, x2: pb.x, y2: pb.y, color });
    }
    lines
}

fn parse_line_strip(
    msg: &visualization_msgs::msg::Marker,
    color: Color,
    iso: &Isometry3<f64>,
) -> Vec<MarkerLine> {
    let pts: Vec<Point3<f64>> = msg.points.iter()
        .map(|p| iso.transform_point(&Point3::new(p.x, p.y, p.z)))
        .collect();
    strips_to_lines(&[pts], color)
}

fn parse_line_list(
    msg: &visualization_msgs::msg::Marker,
    color: Color,
    iso: &Isometry3<f64>,
) -> Vec<MarkerLine> {
    let mut lines = Vec::new();
    let mut pts = msg.points.iter();
    let mut colors = msg.colors.iter();
    while let Some(mp1) = pts.next() {
        let seg_color = match colors.next() {
            Some(c) => Color::Rgb((c.r * 255.0) as u8, (c.g * 255.0) as u8, (c.b * 255.0) as u8),
            None => color,
        };
        colors.next(); // colors come in pairs
        let p1 = iso.transform_point(&Point3::new(mp1.x, mp1.y, mp1.z));
        if let Some(mp2) = pts.next() {
            let p2 = iso.transform_point(&Point3::new(mp2.x, mp2.y, mp2.z));
            lines.push(MarkerLine { x1: p1.x, y1: p1.y, x2: p2.x, y2: p2.y, color: seg_color });
        }
    }
    lines
}

fn parse_cube_list(
    msg: &visualization_msgs::msg::Marker,
    color: Color,
    iso: &Isometry3<f64>,
) -> Vec<MarkerLine> {
    msg.points.iter()
        .flat_map(|pt| parse_cube(&msg.scale, pt, color, iso))
        .collect()
}

fn parse_marker(
    msg: &visualization_msgs::msg::Marker,
    tf_iso: Isometry3<f64>,
) -> Vec<MarkerLine> {
    let marker_iso = iso_from_pose(&msg.pose);
    let iso = tf_iso * marker_iso;

    let color = Color::Rgb(
        (msg.color.r * 255.0) as u8,
        (msg.color.g * 255.0) as u8,
        (msg.color.b * 255.0) as u8,
    );

    match msg.type_ {
        0 => parse_arrow(msg, color, &iso),                // ARROW
        1 => {                                              // CUBE
            let center = geometry_msgs::msg::Point { x: 0.0, y: 0.0, z: 0.0 };
            parse_cube(&msg.scale, &center, color, &iso)
        }
        2 => parse_sphere(msg, color, &iso),               // SPHERE
        4 => parse_line_strip(msg, color, &iso),           // LINE_STRIP
        5 => parse_line_list(msg, color, &iso),            // LINE_LIST
        6 => parse_cube_list(msg, color, &iso),            // CUBE_LIST
        7 => {                                             // SPHERE_LIST
            msg.points.iter()
                .flat_map(|pt| {
                    let sphere_iso = Isometry3::from_parts(
                        Translation3::new(pt.x, pt.y, pt.z),
                        UnitQuaternion::identity(),
                    );
                    let full_iso = iso * sphere_iso;
                    parse_sphere(msg, color, &full_iso)
                })
                .collect()
        }
        8 => {                                             // POINTS (draw as tiny cubes)
            parse_cube_list(msg, color, &iso)
        }
        _ => Vec::new(),
    }
}

// ── process one marker action ─────────────────────────────────────────────────

fn handle_marker(
    store: &mut MarkerStore,
    msg: &visualization_msgs::msg::Marker,
    tf_iso: Isometry3<f64>,
) {
    match msg.action {
        0 => {  // ADD / MODIFY
            let lines = parse_marker(msg, tf_iso);
            store.add(
                msg.ns.clone(),
                msg.id,
                lines,
                msg.lifetime.sec,
                msg.lifetime.nanosec,
            );
        }
        2 => store.delete(&msg.ns, msg.id),  // DELETE
        3 => store.clear(),                   // DELETEALL
        _ => {}
    }
}

// ── listeners ─────────────────────────────────────────────────────────────────

pub struct MarkerListener {
    _config: ListenerConfig,
    pub store: Arc<RwLock<MarkerStore>>,
    _sub: Arc<dyn std::any::Any + Send + Sync>,
}

impl MarkerListener {
    pub fn new(
        config: ListenerConfig,
        ros: Arc<ROS>,
        fixed_frame: String,
    ) -> Result<Self, RclrsError> {
        let store = Arc::new(RwLock::new(MarkerStore::new()));
        let store_cb = Arc::clone(&store);
        let tf = Arc::clone(&ros.tf);
        let frames = Arc::clone(&ros.frames);

        let sub = ros.node.create_subscription::<visualization_msgs::msg::Marker, _>(
            SubscriptionOptions::new(&config.topic),
            move |msg: visualization_msgs::msg::Marker| {
                let tf_iso: Isometry3<f64> = if msg.header.frame_id == fixed_frame {
                    Isometry3::identity()
                } else {
                    let stamp = {
                        let frm = frames.lock().unwrap();
                        match frm.values().filter_map(|(_, s)| *s).min() {
                            Some(s) => s,
                            None => return,
                        }
                    };
                    let result = tf.lock().unwrap().get_transform(&fixed_frame, &msg.header.frame_id, stamp);
                    let Ok(t) = result else { return };
                    tf_iso_from_transform(&t)
                };
                handle_marker(&mut store_cb.write().unwrap(), &msg, tf_iso);
            },
        )?;

        Ok(MarkerListener { _config: config, store, _sub: Arc::new(sub) })
    }
}

pub struct MarkerArrayListener {
    _config: ListenerConfig,
    pub store: Arc<RwLock<MarkerStore>>,
    _sub: Arc<dyn std::any::Any + Send + Sync>,
}

impl MarkerArrayListener {
    pub fn new(
        config: ListenerConfig,
        ros: Arc<ROS>,
        fixed_frame: String,
    ) -> Result<Self, RclrsError> {
        let store = Arc::new(RwLock::new(MarkerStore::new()));
        let store_cb = Arc::clone(&store);
        let tf = Arc::clone(&ros.tf);
        let frames = Arc::clone(&ros.frames);

        let sub = ros.node.create_subscription::<visualization_msgs::msg::MarkerArray, _>(
            SubscriptionOptions::new(&config.topic),
            move |msg: visualization_msgs::msg::MarkerArray| {
                let mut s = store_cb.write().unwrap();
                for marker in &msg.markers {
                    let tf_iso: Isometry3<f64> = if marker.header.frame_id == fixed_frame {
                        Isometry3::identity()
                    } else {
                        let stamp = {
                            let frm = frames.lock().unwrap();
                            match frm.values().filter_map(|(_, s)| *s).min() {
                                Some(s) => s,
                                None => continue,
                            }
                        };
                        let result = tf.lock().unwrap().get_transform(&fixed_frame, &marker.header.frame_id, stamp);
                        let Ok(t) = result else { continue };
                        tf_iso_from_transform(&t)
                    };
                    handle_marker(&mut s, marker, tf_iso);
                }
            },
        )?;

        Ok(MarkerArrayListener { _config: config, store, _sub: Arc::new(sub) })
    }
}
