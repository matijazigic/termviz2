use crate::{
    config::Termviz2Config,
    inputs::listeners::Listeners,
    modes::{input, BaseMode, image_view::ImageView, plot_view::PlotView, send_pose::SendPose, teleoperate::Teleoperate, topic_management::TopicManager, viewport::Viewport},
    ros::ROS,
    utils,
};
use crossterm::event::KeyCode;
use ratatui::layout::{Alignment, Constraint, Direction, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Cell, Paragraph, Row, Table, Wrap};
use ratatui::Frame;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

pub struct App {
    pub key_to_action: HashMap<KeyCode, String>,
    /// Reverse of key_to_action: action name → key label for display.
    action_to_key: HashMap<String, String>,
    modes: Vec<Box<dyn BaseMode>>,
    active: usize,
    show_help: bool,
}

impl App {
    pub fn new(ros: Arc<ROS>, config: Termviz2Config) -> App {
        let key_to_action = utils::build_key_map(&config.key_mapping);
        let action_to_key: HashMap<String, String> = config.key_mapping.clone();

        let mut listeners = Listeners::new(Arc::clone(&ros), &config);
        let images = std::mem::take(&mut listeners.images);

        let zoom = Arc::new(Mutex::new(1.0f64));

        let viewport = Viewport::new(
            Arc::clone(&ros),
            listeners,
            config.fixed_frame.clone(),
            config.robot_frame.clone(),
            config.visible_area.clone(),
            Arc::clone(&zoom),
            config.zoom_factor,
            config.axis_length,
        );

        let mut modes: Vec<Box<dyn BaseMode>> = vec![Box::new(viewport)];

        if let Some(teleop_cfg) = config.teleop.clone() {
            let tl_listeners = Listeners::new(Arc::clone(&ros), &config);
            let tl_viewport = Viewport::new(
                Arc::clone(&ros),
                tl_listeners,
                config.fixed_frame.clone(),
                config.robot_frame.clone(),
                config.visible_area.clone(),
                Arc::clone(&zoom),
                config.zoom_factor,
                config.axis_length,
            );
            if let Some(teleop) = Teleoperate::new(teleop_cfg, tl_viewport) {
                modes.push(Box::new(teleop));
            }
        }

        if !config.send_pose_topics.is_empty() {
            let sp_listeners = Listeners::new(Arc::clone(&ros), &config);
            let sp_viewport = Viewport::new(
                Arc::clone(&ros),
                sp_listeners,
                config.fixed_frame.clone(),
                config.robot_frame.clone(),
                config.visible_area.clone(),
                Arc::clone(&zoom),
                config.zoom_factor,
                config.axis_length,
            );
            let send_pose = SendPose::new(&config.send_pose_topics, sp_viewport);
            modes.push(Box::new(send_pose));
        }

        modes.push(Box::new(ImageView::new(images)));
        modes.push(Box::new(PlotView::new(Arc::clone(&ros))));
        modes.push(Box::new(TopicManager::new(Arc::clone(&ros), config.clone())));

        App {
            key_to_action,
            action_to_key,
            modes,
            active: 0,
            show_help: false,
        }
    }

    pub fn run(&mut self) {
        self.modes[self.active].run();
    }

    pub fn draw(&self, f: &mut Frame) {
        if self.show_help {
            self.draw_help(f);
        } else {
            self.modes[self.active].draw(f);
        }
    }

    pub fn handle_input(&mut self, action: &str) {
        if action == input::SHOW_HELP {
            self.show_help = !self.show_help;
            return;
        }
        if self.show_help {
            return;
        }
        if let Ok(n) = action.parse::<usize>() {
            self.switch_mode(n.saturating_sub(1));
            return;
        }
        self.modes[self.active].handle_input(action);
    }

    fn switch_mode(&mut self, new: usize) {
        if new < self.modes.len() && new != self.active {
            self.modes[self.active].reset();
            self.active = new;
            self.modes[self.active].reset();
        }
    }

    fn key_label(&self, action: &str) -> String {
        self.action_to_key
            .get(action)
            .cloned()
            .unwrap_or_else(|| action.to_string())
    }

    fn draw_help(&self, f: &mut Frame) {
        let mode = &self.modes[self.active];
        let area = f.area();

        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .horizontal_margin(8)
            .vertical_margin(2)
            .constraints([
                Constraint::Length(3),
                Constraint::Length(2 + mode.get_description().len() as u16),
                Constraint::Min(5),
            ])
            .split(area);

        // Title
        let title = Paragraph::new(Line::from(vec![
            Span::styled("termviz2  —  ", Style::default().fg(Color::DarkGray)),
            Span::styled(
                mode.get_name(),
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
        ]))
        .block(Block::default().borders(Borders::ALL))
        .alignment(Alignment::Center);
        f.render_widget(title, chunks[0]);

        // Description
        let desc_lines: Vec<Line> = mode
            .get_description()
            .into_iter()
            .map(|s| Line::from(Span::raw(s)))
            .collect();
        let description = Paragraph::new(desc_lines)
            .block(Block::default().borders(Borders::ALL))
            .alignment(Alignment::Center)
            .wrap(Wrap { trim: false });
        f.render_widget(description, chunks[1]);

        // Key binding rows
        let mut rows: Vec<Row> = Vec::new();

        // Mode switching
        for (i, m) in self.modes.iter().enumerate() {
            rows.push(Row::new([
                Cell::from(format!("{}", i + 1))
                    .style(Style::default().fg(Color::Yellow)),
                Cell::from(format!("Switch to {} mode", m.get_name())),
            ]));
        }
        rows.push(Row::new([Cell::from(""), Cell::from("")]));

        // Current mode's bindings
        for [action, desc] in mode.get_keymap() {
            let key = self.key_label(&action);
            rows.push(Row::new([
                Cell::from(key).style(Style::default().fg(Color::Yellow)),
                Cell::from(desc),
            ]));
        }
        rows.push(Row::new([Cell::from(""), Cell::from("")]));

        // Global bindings
        rows.push(Row::new([
            Cell::from(self.key_label(input::SHOW_HELP))
                .style(Style::default().fg(Color::Yellow)),
            Cell::from("Toggle this help screen"),
        ]));
        rows.push(Row::new([
            Cell::from("Ctrl+c").style(Style::default().fg(Color::Yellow)),
            Cell::from("Quit"),
        ]));

        let table = Table::new(rows, [Constraint::Length(12), Constraint::Percentage(100)])
            .block(Block::default().title(" Key bindings ").borders(Borders::ALL))
            .header(Row::new([
                Cell::from("Key")
                    .style(Style::default().fg(Color::White).add_modifier(Modifier::BOLD)),
                Cell::from("Action")
                    .style(Style::default().fg(Color::White).add_modifier(Modifier::BOLD)),
            ]))
            .style(Style::default().fg(Color::White))
            .column_spacing(2);
        f.render_widget(table, chunks[2]);
    }
}
