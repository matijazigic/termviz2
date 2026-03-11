pub mod image_view;
pub mod send_pose;
pub mod teleoperate;
pub mod topic_management;
pub mod viewport;

use ratatui::Frame;

pub mod input {
    pub const ZOOM_IN: &str = "zoom_in";
    pub const ZOOM_OUT: &str = "zoom_out";
    pub const LEFT: &str = "left";
    pub const RIGHT: &str = "right";
    pub const UP: &str = "up";
    pub const DOWN: &str = "down";
    pub const NEXT: &str = "next";
    pub const PREVIOUS: &str = "previous";
    pub const SHOW_HELP: &str = "show_help";
    pub const ROTATE_LEFT: &str = "rotate_left";
    pub const ROTATE_RIGHT: &str = "rotate_right";
    pub const CONFIRM: &str = "confirm";
    pub const CANCEL: &str = "cancel";
    pub const INCREMENT_STEP: &str = "increment_step";
    pub const DECREMENT_STEP: &str = "decrement_step";
}

/// Common interface every app mode must implement.
pub trait AppMode {
    fn run(&mut self);
    fn reset(&mut self);
    fn handle_input(&mut self, action: &str);
    fn get_name(&self) -> String;
    fn get_description(&self) -> Vec<String>;
    fn get_keymap(&self) -> Vec<[String; 2]>;
}

/// Something that can be drawn on the terminal.
pub trait Drawable {
    fn draw(&self, f: &mut Frame);
}

/// Marker trait: a full app mode that can also be drawn.
pub trait BaseMode: AppMode + Drawable {}
