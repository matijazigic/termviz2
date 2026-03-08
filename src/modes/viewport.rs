use crate::inputs::listeners::Listeners;
use crate::modes::{input, AppMode, BaseMode, Drawable};
use crate::ros::ROS;
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line as TextLine, Span};
use ratatui::widgets::canvas::{Canvas, Context, Line, Points};
use ratatui::widgets::{Block, Borders};
use ratatui::Frame;
use std::sync::Arc;
use transforms::time::Timestamp;

/// Modes that render inside a canvas viewport implement this.
/// `Drawable` is then automatically provided via the blanket impl below.
pub trait UseViewport: AppMode {
    fn draw_in_viewport(&self, ctx: &mut Context);
    fn x_bounds(&self, scale_factor: f64) -> [f64; 2];
    fn y_bounds(&self) -> [f64; 2];
    fn info(&self) -> String;
}

impl<T: UseViewport> Drawable for T {
    fn draw(&self, f: &mut Frame) {
        let area = f.area();
        let scale_factor = area.width as f64 / area.height as f64 * 0.5;
        let title = TextLine::from(vec![
            Span::styled(
                self.get_name(),
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::raw(" - "),
            Span::raw(self.info()),
        ]);
        let canvas = Canvas::default()
            .block(Block::default().title(title).borders(Borders::NONE))
            .x_bounds(self.x_bounds(scale_factor))
            .y_bounds(self.y_bounds())
            .paint(|ctx| self.draw_in_viewport(ctx));
        f.render_widget(canvas, area);
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
        vec!["Visualizes the robot and map in the terminal.".to_string()]
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

    fn info(&self) -> String {
        let (rx, ry) = self.robot_position();
        format!("zoom: {:.1}x | robot: ({:.2}, {:.2})", self.zoom, rx, ry)
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
        // Layer 1: footprints
        for fp in &self.listeners.footprints {
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

        ctx.layer();
        // Layer 3: robot axes
        let iso = self.robot_iso();
        let origin = iso.translation.vector;
        let x_tip = iso.transform_point(&Point3::new(self.axis_length, 0.0, 0.0));
        let y_tip = iso.transform_point(&Point3::new(0.0, self.axis_length, 0.0));

        ctx.draw(&Line {
            x1: origin.x,
            y1: origin.y,
            x2: y_tip.x,
            y2: y_tip.y,
            color: Color::Green,
        });
        ctx.draw(&Line {
            x1: origin.x,
            y1: origin.y,
            x2: x_tip.x,
            y2: x_tip.y,
            color: Color::Red,
        });
    }
}

impl BaseMode for Viewport {}
