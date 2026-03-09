use crate::config::PoseStyle;
use crate::inputs::listeners::Listeners;
use crate::modes::{input, AppMode, BaseMode, Drawable};
use crate::ros::ROS;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line as TextLine, Span};
use ratatui::layout::Rect;
use ratatui::text::Text;
use ratatui::widgets::canvas::{Canvas, Context, Line, Points};
use ratatui::widgets::{Block, Borders, Paragraph};
use ratatui::Frame;
use std::sync::Arc;
use transforms::time::Timestamp;

fn draw_arrow(ctx: &mut Context, x: f64, y: f64, yaw: f64, length: f64, color: Color) {
    let (sin, cos) = yaw.sin_cos();
    let tip_x = x + length * cos;
    let tip_y = y + length * sin;
    ctx.draw(&Line { x1: x, y1: y, x2: tip_x, y2: tip_y, color });
    let lx = x + length * 0.5 * cos - length * 0.25 * sin;
    let ly = y + length * 0.5 * sin + length * 0.25 * cos;
    ctx.draw(&Line { x1: tip_x, y1: tip_y, x2: lx, y2: ly, color });
    let rx = x + length * 0.5 * cos + length * 0.25 * sin;
    let ry = y + length * 0.5 * sin - length * 0.25 * cos;
    ctx.draw(&Line { x1: tip_x, y1: tip_y, x2: rx, y2: ry, color });
}

fn draw_axes(ctx: &mut Context, x: f64, y: f64, yaw: f64, length: f64) {
    let (sin, cos) = yaw.sin_cos();
    ctx.draw(&Line { x1: x, y1: y, x2: x + length * cos, y2: y + length * sin, color: Color::Red });
    ctx.draw(&Line { x1: x, y1: y, x2: x - length * sin, y2: y + length * cos, color: Color::Green });
}

fn draw_pose_lines(ctx: &mut Context, poses: &[(f64, f64, f64)], color: Color) {
    for w in poses.windows(2) {
        ctx.draw(&Line { x1: w[0].0, y1: w[0].1, x2: w[1].0, y2: w[1].1, color });
    }
}

/// Modes that render inside a canvas viewport implement this.
/// `Drawable` is then automatically provided via the blanket impl below.
pub trait UseViewport: AppMode {
    fn draw_in_viewport(&self, ctx: &mut Context);
    fn x_bounds(&self, scale_factor: f64) -> [f64; 2];
    fn y_bounds(&self) -> [f64; 2];
    fn draw_overlay(&self, _f: &mut Frame, _area: Rect) {}
}

impl<T: UseViewport> Drawable for T {
    fn draw(&self, f: &mut Frame) {
        let area = f.area();
        let scale_factor = area.width as f64 / area.height as f64 * 0.5;
        let title = TextLine::from(vec![
            Span::styled(
                self.get_name(),
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            )
        ]);
        let canvas = Canvas::default()
            .block(Block::default().title(title).borders(Borders::NONE))
            .x_bounds(self.x_bounds(scale_factor))
            .y_bounds(self.y_bounds())
            .paint(|ctx| self.draw_in_viewport(ctx));
        f.render_widget(canvas, area);
        self.draw_overlay(f, area);
    }
}

pub struct Viewport {
    pub ros: Arc<ROS>,
    pub listeners: Listeners,
    pub fixed_frame: String,
    pub robot_frame: String,
    pub initial_bounds: Vec<f64>, // [x_min, x_max, y_min, y_max]
    pub zoom: f64,
    pub zoom_factor: f64,
    pub axis_length: f64,
}

impl Viewport {
    pub fn new(
        ros: Arc<ROS>,
        listeners: Listeners,
        fixed_frame: String,
        robot_frame: String,
        initial_bounds: Vec<f64>,
        zoom_factor: f64,
        axis_length: f64,
    ) -> Self {
        Viewport {
            ros,
            listeners,
            fixed_frame,
            robot_frame,
            initial_bounds,
            zoom: 1.0,
            zoom_factor,
            axis_length,
        }
    }

    fn robot_position(&self) -> (f64, f64) {
        let stamp = self.ros.min_dynamic_stamp().unwrap_or(Timestamp { t: 0 });
        match self.ros.tf.lock().unwrap().get_transform(&self.fixed_frame, &self.robot_frame, stamp) {
            Ok(t) => (t.translation.x, t.translation.y),
            Err(_) => (0.0, 0.0),
        }
    }

    fn robot_iso(&self) -> Isometry3<f64> {
        let stamp = self.ros.min_dynamic_stamp().unwrap_or(Timestamp { t: 0 });
        match self.ros.tf.lock().unwrap().get_transform(&self.fixed_frame, &self.robot_frame, stamp) {
            Ok(t) => {
                let tra = Translation3::new(t.translation.x, t.translation.y, t.translation.z);
                let rot = UnitQuaternion::new_normalize(Quaternion::new(
                    t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z,
                ));
                Isometry3::from_parts(tra, rot)
            }
            Err(_) => Isometry3::identity(),
        }
    }
}

impl AppMode for Viewport {
    fn run(&mut self) {}

    fn reset(&mut self) {
        self.zoom = 1.0;
    }

    fn handle_input(&mut self, action: &str) {
        match action {
            input::ZOOM_IN => self.zoom += self.zoom_factor,
            input::ZOOM_OUT => {
                self.zoom -= self.zoom_factor;
                if self.zoom < 0.1 {
                    self.zoom = 0.1;
                }
            }
            _ => {}
        }
    }

    fn get_name(&self) -> String {
        "Viewport".to_string()
    }

    fn get_description(&self) -> Vec<String> {
        vec!["Visualizes the robot, map and defined topics in the terminal.".to_string()]
    }

    fn get_keymap(&self) -> Vec<[String; 2]> {
        vec![
            [input::ZOOM_IN.to_string(), "Zoom in".to_string()],
            [input::ZOOM_OUT.to_string(), "Zoom out".to_string()],
        ]
    }
}

impl UseViewport for Viewport {
    fn x_bounds(&self, scale_factor: f64) -> [f64; 2] {
        let (rx, _) = self.robot_position();
        [
            rx + self.initial_bounds[0] / self.zoom * scale_factor,
            rx + self.initial_bounds[1] / self.zoom * scale_factor,
        ]
    }

    fn y_bounds(&self) -> [f64; 2] {
        let (_, ry) = self.robot_position();
        [
            ry + self.initial_bounds[2] / self.zoom,
            ry + self.initial_bounds[3] / self.zoom,
        ]
    }

    fn draw_overlay(&self, f: &mut Frame, area: Rect) {
        let (rx, ry) = self.robot_position();
        let lines = vec![
            TextLine::from(vec![
                Span::styled("pos  ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:.2}, {:.2}", rx, ry)),
            ]),
            TextLine::from(vec![
                Span::styled("zoom ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:.1}x", self.zoom)),
            ]),
        ];
        let width = 20u16;
        let height = lines.len() as u16 + 2;
        let overlay = Rect {
            x: area.width.saturating_sub(width + 1),
            y: 1,
            width,
            height,
        };
        let paragraph = Paragraph::new(Text::from(lines))
            .block(Block::default().borders(Borders::ALL))
            .style(Style::default().fg(Color::White));
        f.render_widget(paragraph, overlay);
    }

    fn draw_in_viewport(&self, ctx: &mut Context) {
        // Layer 0: maps
        for map in &self.listeners.maps {
            let pts = map.points.read().unwrap();
            ctx.draw(&Points {
                coords: &pts,
                color: Color::Rgb(map.config.color.r, map.config.color.g, map.config.color.b),
            });
        }

        ctx.layer();
        // Layer 1: polygons
        for fp in &self.listeners.polygons {
            let lines = fp.lines.read().unwrap();
            for &(x1, y1, x2, y2) in lines.iter() {
                ctx.draw(&Line {
                    x1, y1, x2, y2,
                    color: Color::Rgb(fp.config.color.r, fp.config.color.g, fp.config.color.b),
                });
            }
        }

        ctx.layer();
        // Layer 2: lasers
        for laser in &self.listeners.lasers {
            let pts = laser.points.read().unwrap();
            ctx.draw(&Points {
                coords: &pts,
                color: Color::Rgb(laser.config.color.r, laser.config.color.g, laser.config.color.b),
            });
        }

        // Layer 3: pointclouds (per-point color)
        for pc in &self.listeners.pointclouds {
            let pts = pc.points.read().unwrap();
            for pt in pts.iter() {
                ctx.draw(&Points { coords: &[(pt.x, pt.y)], color: pt.color });
            }
        }

        ctx.layer();
        // Layer 4: pose stamped
        for listener in &self.listeners.poses {
            if let Some((x, y, yaw)) = *listener.pose.read().unwrap() {
                let color = Color::Rgb(listener.config.color.r, listener.config.color.g, listener.config.color.b);
                let len = listener.config.length;
                match listener.config.style {
                    PoseStyle::Arrow => draw_arrow(ctx, x, y, yaw, len, color),
                    PoseStyle::Axes  => draw_axes(ctx, x, y, yaw, len),
                    PoseStyle::Line  => draw_arrow(ctx, x, y, yaw, len, color),
                }
            }
        }

        ctx.layer();
        // Layer 5: pose arrays
        for listener in &self.listeners.pose_arrays {
            let color = Color::Rgb(listener.config.color.r, listener.config.color.g, listener.config.color.b);
            let len = listener.config.length;
            let poses = listener.poses.read().unwrap();
            match listener.config.style {
                PoseStyle::Arrow => {
                    for &(x, y, yaw) in poses.iter() {
                        draw_arrow(ctx, x, y, yaw, len, color);
                    }
                }
                PoseStyle::Axes => {
                    for &(x, y, yaw) in poses.iter() {
                        draw_axes(ctx, x, y, yaw, len);
                    }
                }
                PoseStyle::Line => draw_pose_lines(ctx, &poses, color),
            }
        }

        ctx.layer();
        // Layer 6: robot axes
        let iso = self.robot_iso();
        let (_, _, yaw) = iso.rotation.euler_angles();
        draw_axes(ctx, iso.translation.x, iso.translation.y, yaw, self.axis_length);

        ctx.layer();
        // Layer 7: markers and marker arrays
        for listener in &self.listeners.markers {
            for ml in listener.store.write().unwrap().get_lines() {
                ctx.draw(&Line { x1: ml.x1, y1: ml.y1, x2: ml.x2, y2: ml.y2, color: ml.color });
            }
        }
        for listener in &self.listeners.marker_arrays {
            for ml in listener.store.write().unwrap().get_lines() {
                ctx.draw(&Line { x1: ml.x1, y1: ml.y1, x2: ml.x2, y2: ml.y2, color: ml.color });
            }
        }

        ctx.layer();
        // Layer 8: paths
        for path in &self.listeners.paths {
            let color = Color::Rgb(path.config.color.r, path.config.color.g, path.config.color.b);
            let len = path.config.length;
            let lines = path.lines.read().unwrap();
            match path.config.style {
                PoseStyle::Arrow => {
                    for &(x1, y1, x2, y2) in lines.iter() {
                        ctx.draw(&Line { x1, y1, x2, y2, color });
                        let dx = x2 - x1;
                        let dy = y2 - y1;
                        let yaw = dy.atan2(dx);
                        draw_arrow(ctx, x1, y1, yaw, len, color);
                    }
                }
                PoseStyle::Line | PoseStyle::Axes => {
                    for &(x1, y1, x2, y2) in lines.iter() {
                        ctx.draw(&Line { x1, y1, x2, y2, color });
                    }
                }
            }
        }
    }
}

impl BaseMode for Viewport {}
