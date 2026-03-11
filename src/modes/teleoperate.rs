//! Teleoperate mode — publishes Twist commands to drive the robot.

use crate::config::{TeleopConfig, TeleopMode};
use crate::modes::{input, AppMode, BaseMode};
use crate::modes::viewport::{UseViewport, Viewport};
use crate::outputs::publishers::CmdVelPub;
use ratatui::layout::Rect;
use ratatui::style::{Color, Style};
use ratatui::text::{Line as TextLine, Span};
use ratatui::widgets::canvas::Context;
use ratatui::widgets::{Block, Borders, Paragraph};
use ratatui::Frame;

fn move_towards_zero(val: f64, step: f64) -> f64 {
    if val > 0.0 {
        (val - step).max(0.0)
    } else {
        (val + step).min(0.0)
    }
}

pub struct Teleoperate {
    viewport: Viewport,
    vx: f64,
    vy: f64,
    wz: f64,
    increment: f64,
    increment_step: f64,
    publish_when_idle: bool,
    has_published_zero: bool,
    mode: TeleopMode,
    max_vel: f64,
    publisher: CmdVelPub,
}

impl Teleoperate {
    pub fn new(config: TeleopConfig, viewport: Viewport) -> Option<Self> {
        match CmdVelPub::new(&viewport.ros, &config.cmd_vel_topic) {
            Ok(publisher) => Some(Teleoperate {
                viewport,
                vx: 0.0,
                vy: 0.0,
                wz: 0.0,
                increment: config.default_increment,
                increment_step: config.increment_step,
                publish_when_idle: config.publish_cmd_vel_when_idle,
                has_published_zero: true,
                mode: config.mode,
                max_vel: config.max_vel,
                publisher,
            }),
            Err(e) => {
                eprintln!("Failed to create cmd_vel publisher for '{}': {:?}", config.cmd_vel_topic, e);
                None
            }
        }
    }

    fn publish(&self) {
        self.publisher.send(self.vx, self.vy, self.wz);
    }

    fn is_zero(&self) -> bool {
        self.vx == 0.0 && self.vy == 0.0 && self.wz == 0.0
    }
}

impl AppMode for Teleoperate {
    fn run(&mut self) {
        if self.mode == TeleopMode::Safe {
            self.vx = move_towards_zero(self.vx, self.increment);
            self.vy = move_towards_zero(self.vy, self.increment);
            self.wz = move_towards_zero(self.wz, self.increment);
        }

        if self.is_zero() && !self.publish_when_idle {
            if !self.has_published_zero {
                self.has_published_zero = true;
                self.publish();
            }
        } else {
            self.has_published_zero = false;
            self.publish();
        }
    }

    fn reset(&mut self) {
        self.vx = 0.0;
        self.vy = 0.0;
        self.wz = 0.0;
        self.run();
    }

    fn handle_input(&mut self, action: &str) {
        // Zoom is always forwarded to the inner viewport.
        self.viewport.handle_input(action);

        match action {
            input::UP => {
                self.vx = (self.vx + self.increment).min(self.max_vel);
            }
            input::DOWN => {
                self.vx = (self.vx - self.increment).max(-self.max_vel);
            }
            input::LEFT => {
                self.vy = (self.vy + self.increment).min(self.max_vel);
            }
            input::RIGHT => {
                self.vy = (self.vy - self.increment).max(-self.max_vel);
            }
            input::ROTATE_LEFT => {
                self.wz = (self.wz + self.increment).min(self.max_vel);
            }
            input::ROTATE_RIGHT => {
                self.wz = (self.wz - self.increment).max(-self.max_vel);
            }
            input::INCREMENT_STEP => {
                self.increment += self.increment_step;
            }
            input::DECREMENT_STEP => {
                self.increment = (self.increment - self.increment_step).max(self.increment_step);
            }
            // Any other key resets velocities (only meaningful for Classic mode;
            // Safe mode decays naturally).
            _ => {
                if self.mode == TeleopMode::Classic {
                    self.reset();
                }
            }
        }
    }

    fn get_name(&self) -> String {
        "Teleoperate".to_string()
    }

    fn get_description(&self) -> Vec<String> {
        vec![
            "Publish velocity commands to drive the robot.".to_string(),
            "Classic: velocities accumulate; any unmapped key resets.".to_string(),
            "Safe: velocities decay to zero each tick unless a key is held.".to_string(),
        ]
    }

    fn get_keymap(&self) -> Vec<[String; 2]> {
        let mut keymap = vec![
            [input::UP.to_string(),             "Increase linear X (forward)".to_string()],
            [input::DOWN.to_string(),           "Decrease linear X (backward)".to_string()],
            [input::LEFT.to_string(),           "Increase linear Y (left)".to_string()],
            [input::RIGHT.to_string(),          "Decrease linear Y (right)".to_string()],
            [input::ROTATE_LEFT.to_string(),    "Rotate counter-clockwise".to_string()],
            [input::ROTATE_RIGHT.to_string(),   "Rotate clockwise".to_string()],
            [input::INCREMENT_STEP.to_string(), "Increase velocity step".to_string()],
            [input::DECREMENT_STEP.to_string(), "Decrease velocity step".to_string()],
        ];
        keymap.extend(self.viewport.get_keymap());
        keymap
    }
}

impl UseViewport for Teleoperate {
    fn draw_in_viewport(&self, ctx: &mut Context) {
        self.viewport.draw_in_viewport(ctx);
    }

    fn x_bounds(&self, scale_factor: f64) -> [f64; 2] {
        self.viewport.x_bounds(scale_factor)
    }

    fn y_bounds(&self) -> [f64; 2] {
        self.viewport.y_bounds()
    }

    fn draw_overlay(&self, f: &mut Frame, area: Rect) {
        let mode_str = match self.mode {
            TeleopMode::Classic => "classic",
            TeleopMode::Safe    => "safe",
        };

        let mut lines = vec![
            TextLine::from(vec![
                Span::styled("mode  ", Style::default().fg(Color::DarkGray)),
                Span::raw(mode_str),
            ]),
            TextLine::from(vec![
                Span::styled("step  ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:.2}", self.increment)),
            ]),
            TextLine::from(vec![
                Span::styled("vx    ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:.2}", self.vx)),
            ]),
            TextLine::from(vec![
                Span::styled("vy    ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:.2}", self.vy)),
            ]),
            TextLine::from(vec![
                Span::styled("wz    ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:.2}", self.wz)),
            ]),
        ];

        if !self.viewport.listeners.odoms.is_empty() {
            lines.push(TextLine::from(Span::styled("odom", Style::default().fg(Color::DarkGray))));
        }
        for odom in &self.viewport.listeners.odoms {
            let (ovx, ovy, owz) = *odom.vel.read().unwrap();
            lines.push(TextLine::from(vec![
                Span::styled("vx    ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:.2}", ovx)),
            ]));
            lines.push(TextLine::from(vec![
                Span::styled("vy    ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:.2}", ovy)),
            ]));
            lines.push(TextLine::from(vec![
                Span::styled("wz    ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:.2}", owz)),
            ]));
        }

        let width = 22u16;
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

        // Bottom-right: config info
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

impl BaseMode for Teleoperate {}
