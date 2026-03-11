//! Send pose mode — lets the user place and publish a pose on a configured topic.

use crate::config::{Color as CfgColor, SendPoseConfig};
use crate::modes::{input, AppMode, BaseMode};
use crate::modes::viewport::{UseViewport, Viewport};
use crate::outputs::publishers::{BasePosePub, create_pose_publishers};
use ratatui::layout::Rect;
use ratatui::style::{Color, Style};
use ratatui::text::{Line as TextLine, Span};
use ratatui::widgets::canvas::{Context, Line};
use ratatui::widgets::{Block, Borders, Paragraph};
use ratatui::Frame;
use transforms::time::Timestamp;

fn quat_to_yaw(w: f64, x: f64, y: f64, z: f64) -> f64 {
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

fn pose_changed(a: (f64, f64, f64), b: (f64, f64, f64), eps: f64) -> bool {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    let dyaw = (b.2 - a.2).abs();
    dx * dx + dy * dy > eps * eps || dyaw > eps
}

fn draw_axes(ctx: &mut Context, x: f64, y: f64, yaw: f64, length: f64, cx: Color, cy: Color) {
    let (sin, cos) = yaw.sin_cos();
    ctx.draw(&Line { x1: x, y1: y, x2: x + length * cos, y2: y + length * sin, color: cx });
    ctx.draw(&Line { x1: x, y1: y, x2: x - length * sin, y2: y + length * cos, color: cy });
}

fn robot_pose_from_viewport(viewport: &Viewport) -> (f64, f64, f64) {
    let stamp = viewport.ros.min_dynamic_stamp().unwrap_or(Timestamp { t: 0 });
    match viewport.ros.tf.lock().unwrap()
        .get_transform(&viewport.fixed_frame, &viewport.robot_frame, stamp)
    {
        Ok(t) => {
            let yaw = quat_to_yaw(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
            (t.translation.x, t.translation.y, yaw)
        }
        Err(_) => (0.0, 0.0, 0.0),
    }
}

struct FootprintConfig {
    topic: String,
    color: CfgColor,
}

/// Send pose mode: renders the full viewport and overlays a movable ghost pose.
pub struct SendPose {
    viewport: Viewport,
    increment: f64,
    robot_pose: (f64, f64, f64),
    new_pose: (f64, f64, f64),
    current_topic: usize,
    publishers: Vec<Box<dyn BasePosePub>>,
    footprints: Vec<Option<FootprintConfig>>,
    ghost_active: bool,
}

impl SendPose {
    pub fn new(configs: &[SendPoseConfig], viewport: Viewport) -> Self {
        let publishers = create_pose_publishers(&viewport.ros, configs);
        let footprints = configs.iter().map(|c| {
            c.footprint_topic.as_ref().map(|t| FootprintConfig {
                topic: t.clone(),
                color: c.footprint_color.clone(),
            })
        }).collect();
        let robot_pose = robot_pose_from_viewport(&viewport);
        SendPose {
            viewport,
            increment: 0.1,
            robot_pose,
            new_pose: robot_pose,
            current_topic: 0,
            publishers,
            footprints,
            ghost_active: false,
        }
    }

    fn move_new_pose(&mut self, dx_body: f64, dy_body: f64, dyaw: f64) {
        let new_yaw = self.new_pose.2 + dyaw;
        let new_x = dx_body * new_yaw.cos() - dy_body * new_yaw.sin() + self.new_pose.0;
        let new_y = dx_body * new_yaw.sin() + dy_body * new_yaw.cos() + self.new_pose.1;
        self.new_pose = (new_x, new_y, new_yaw);
        self.ghost_active = true;
        
    }

    fn send_new_pose(&mut self) {
        if self.publishers.is_empty() {
            return;
        }
        if pose_changed(self.robot_pose, self.new_pose, 0.01) {
            let (x, y, yaw) = self.new_pose;
            let frame_id = self.viewport.fixed_frame.clone();
            self.publishers[self.current_topic].send(x, y, yaw, &frame_id);
            self.ghost_active = false;
        }
    }

    fn center(&self) -> (f64, f64) {
        if self.ghost_active {
            let (rx, ry, _) = self.robot_pose;
            let (gx, gy, _) = self.new_pose;
            ((rx + gx) * 0.5, (ry + gy) * 0.5)
        } else {
            (self.robot_pose.0, self.robot_pose.1)
        }
    }
}

impl AppMode for SendPose {
    fn run(&mut self) {
        self.robot_pose = robot_pose_from_viewport(&self.viewport);
        if !self.ghost_active {
            self.new_pose = self.robot_pose;
        }
    }

    fn reset(&mut self) {
        self.ghost_active = false;
        self.viewport.reset();
        self.run();
    }

    fn handle_input(&mut self, action: &str) {
        match action {
            input::UP           => self.move_new_pose(self.increment, 0.0, 0.0),
            input::DOWN         => self.move_new_pose(-self.increment, 0.0, 0.0),
            input::LEFT         => self.move_new_pose(0.0, self.increment, 0.0),
            input::RIGHT        => self.move_new_pose(0.0, -self.increment, 0.0),
            input::ROTATE_LEFT  => self.move_new_pose(0.0, 0.0, self.increment),
            input::ROTATE_RIGHT => self.move_new_pose(0.0, 0.0, -self.increment),
            input::INCREMENT_STEP => self.increment += 0.1,
            input::DECREMENT_STEP => {
                self.increment = (self.increment - 0.1).max(0.1);
            }
            input::NEXT => {
                if !self.publishers.is_empty() {
                    self.current_topic = (self.current_topic + 1) % self.publishers.len();
                }
            }
            input::PREVIOUS => {
                if !self.publishers.is_empty() {
                    self.current_topic = if self.current_topic > 0 {
                        self.current_topic - 1
                    } else {
                        self.publishers.len() - 1
                    };
                }
            }
            input::CANCEL  => self.reset(),
            input::CONFIRM => self.send_new_pose(),
            // Zoom is delegated to the inner viewport.
            _ => self.viewport.handle_input(action),
        }
    }

    fn get_name(&self) -> String {
        "Send Pose".to_string()
    }

    fn get_description(&self) -> Vec<String> {
        vec![
            "Publish a pose message on a configured topic.".to_string(),
            "Move the ghost pose with the directional keys, then confirm to send.".to_string(),
        ]
    }

    fn get_keymap(&self) -> Vec<[String; 2]> {
        let mut keymap = vec![
            [input::UP.to_string(),             "Move ghost pose +X (robot frame)".to_string()],
            [input::DOWN.to_string(),           "Move ghost pose -X (robot frame)".to_string()],
            [input::LEFT.to_string(),           "Move ghost pose +Y (robot frame)".to_string()],
            [input::RIGHT.to_string(),          "Move ghost pose -Y (robot frame)".to_string()],
            [input::ROTATE_LEFT.to_string(),    "Rotate ghost pose counter-clockwise".to_string()],
            [input::ROTATE_RIGHT.to_string(),   "Rotate ghost pose clockwise".to_string()],
            [input::INCREMENT_STEP.to_string(), "Increase step size".to_string()],
            [input::DECREMENT_STEP.to_string(), "Decrease step size".to_string()],
            [input::NEXT.to_string(),           "Switch to next topic".to_string()],
            [input::PREVIOUS.to_string(),       "Switch to previous topic".to_string()],
            [input::CONFIRM.to_string(),        "Send the ghost pose".to_string()],
            [input::CANCEL.to_string(),         "Reset the ghost pose".to_string()],
        ];
        keymap.extend(self.viewport.get_keymap());
        keymap
    }
}

impl UseViewport for SendPose {
    fn draw_in_viewport(&self, ctx: &mut Context) {
        // Draw the full viewport (map, laser, polygons, robot axes, etc.)
        self.viewport.draw_in_viewport(ctx);

        // Overlay the ghost pose on top
        if self.ghost_active {
            let (rx, ry, _) = robot_pose_from_viewport(&self.viewport);
            let (gx, gy, gyaw) = self.new_pose;
            let (sin, cos) = gyaw.sin_cos();

            ctx.layer();
            draw_axes(ctx, gx, gy, gyaw, self.viewport.axis_length, Color::Gray, Color::DarkGray);
            ctx.draw(&Line { x1: rx, y1: ry, x2: gx, y2: gy, color: Color::DarkGray });

            // Draw ghost footprint from the per-topic footprint config
            if let Some(Some(fp_cfg)) = self.footprints.get(self.current_topic) {
                if let Some(fp) = self.viewport.listeners.polygons.iter()
                    .find(|p| p.config.topic == fp_cfg.topic)
                {
                    let pts = fp.raw_points.read().unwrap();
                    if pts.len() >= 2 {
                        let world: Vec<(f64, f64)> = pts.iter().map(|(px, py)| {
                            (px * cos - py * sin + gx, px * sin + py * cos + gy)
                        }).collect();
                        let c = &fp_cfg.color;
                        let color = Color::Rgb(c.r, c.g, c.b);
                        for i in 0..world.len() {
                            let p0 = world[i];
                            let p1 = world[(i + 1) % world.len()];
                            ctx.draw(&Line { x1: p0.0, y1: p0.1, x2: p1.0, y2: p1.1, color });
                        }
                    }
                }
            }
        }
    }

    fn x_bounds(&self, scale_factor: f64) -> [f64; 2] {
        let (cx, _) = self.center();
        let zoom = *self.viewport.zoom.lock().unwrap();
        [
            cx + self.viewport.initial_bounds[0] / zoom * scale_factor,
            cx + self.viewport.initial_bounds[1] / zoom * scale_factor,
        ]
    }

    fn y_bounds(&self) -> [f64; 2] {
        let (_, cy) = self.center();
        let zoom = *self.viewport.zoom.lock().unwrap();
        [
            cy + self.viewport.initial_bounds[2] / zoom,
            cy + self.viewport.initial_bounds[3] / zoom,
        ]
    }

    fn draw_overlay(&self, f: &mut Frame, area: Rect) {
        let topic_str = if self.publishers.is_empty() {
            "no topics configured".to_string()
        } else {
            format!(
                "{} ({}/{})",
                self.publishers[self.current_topic].get_topic(),
                self.current_topic + 1,
                self.publishers.len()
            )
        };

        let lines = vec![
            TextLine::from(vec![
                Span::styled("topic ", Style::default().fg(Color::DarkGray)),
                Span::raw(topic_str),
            ]),
            TextLine::from(vec![
                Span::styled("step  ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:.2} m", self.increment)),
            ]),
        ];
        let width = 30u16;
        let height = lines.len() as u16 + 2;
        let overlay = Rect {
            x: area.x + area.width.saturating_sub(width + 1),
            y: area.y,
            width,
            height,
        };
        let paragraph = Paragraph::new(lines)
            .block(Block::default().title(" Info ").borders(Borders::ALL))
            .style(Style::default().fg(Color::White));
        f.render_widget(paragraph, overlay);

        // Bottom-right: general info
        let zoom = *self.viewport.zoom.lock().unwrap();
        let general_lines = vec![
            TextLine::from(vec![
                Span::styled("zoom        ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:.1}x", zoom)),
            ]),
            TextLine::from(vec![
                Span::styled("zoom factor ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:.2}", self.viewport.zoom_factor)),
            ]),
        ];
        let bot_height = general_lines.len() as u16 + 2;
        let bot_overlay = Rect {
            x: area.x + area.width.saturating_sub(width + 1),
            y: (area.y + area.height).saturating_sub(bot_height + 1),
            width,
            height: bot_height,
        };
        let bot_paragraph = Paragraph::new(general_lines)
            .block(Block::default().title(" Config").borders(Borders::ALL))
            .style(Style::default().fg(Color::White));
        f.render_widget(bot_paragraph, bot_overlay);
    }
}

impl BaseMode for SendPose {}
