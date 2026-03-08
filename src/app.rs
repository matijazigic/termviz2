use crate::{
    config::Termviz2Config,
    inputs::listeners::Listeners,
    modes::{input, AppMode, BaseMode, Drawable, viewport::Viewport},
    ros::ROS,
    utils,
};
use crossterm::event::KeyCode;
use ratatui::Frame;
use std::collections::HashMap;
use std::sync::Arc;

pub struct App {
    pub key_to_action: HashMap<KeyCode, String>,
    modes: Vec<Box<dyn BaseMode>>,
    active: usize,
}

impl App {
    pub fn new(ros: Arc<ROS>, config: Termviz2Config) -> App {
        let key_to_action = utils::build_key_map(&config.key_mapping);
        let listeners = Listeners::new(Arc::clone(&ros), &config);
        let viewport = Viewport::new(
            Arc::clone(&ros),
            listeners,
            config.fixed_frame.clone(),
            config.robot_frame.clone(),
            config.visible_area.clone(),
            config.zoom_factor,
            config.axis_length,
        );

        App {
            key_to_action,
            modes: vec![Box::new(viewport)],
            active: 0,
        }
    }

    pub fn run(&mut self) {
        self.modes[self.active].run();
    }

    pub fn draw(&self, f: &mut Frame) {
        self.modes[self.active].draw(f);
    }

    pub fn handle_input(&mut self, action: &str) {
        match action {
            input::NEXT => self.switch_mode(self.active + 1),
            input::PREVIOUS => self.switch_mode(self.active.saturating_sub(1)),
            _ => self.modes[self.active].handle_input(action),
        }
    }

    fn switch_mode(&mut self, new: usize) {
        if new < self.modes.len() && new != self.active {
            self.modes[self.active].reset();
            self.active = new;
            self.modes[self.active].reset();
        }
    }
}
