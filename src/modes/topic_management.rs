use crate::config::{
    Color as ConfigColor, ImageListenerConfig, ListenerConfig, ListenerConfigColor,
    MapListenerConfig, PathListenerConfig, PointCloud2ListenerConfig, PoseListenerConfig,
    Termviz2Config,
};
use crate::modes::{input, AppMode, BaseMode, Drawable};
use crate::ros::ROS;
use ratatui::layout::{Alignment, Constraint, Direction, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, List, ListItem, ListState, Paragraph, Wrap};
use ratatui::Frame;
use std::sync::Arc;

// --- supported ROS2 topic types and their config category labels ---------------

const SUPPORTED_TYPES: &[(&str, &str)] = &[
    ("sensor_msgs/msg/LaserScan", "LaserScan"),
    ("sensor_msgs/msg/PointCloud2", "PointCloud2"),
    ("sensor_msgs/msg/Image", "Image"),
    ("geometry_msgs/msg/PoseStamped", "PoseStamped"),
    ("geometry_msgs/msg/PoseArray", "PoseArray"),
    ("nav_msgs/msg/Path", "Path"),
    ("nav_msgs/msg/OccupancyGrid", "OccupancyGrid"),
    ("visualization_msgs/msg/Marker", "Marker"),
    ("visualization_msgs/msg/MarkerArray", "MarkerArray"),
    ("geometry_msgs/msg/PolygonStamped", "PolygonStamped"),
];

/// Deterministic color palette used when adding new topics (no rand dependency).
const COLOR_PALETTE: [(u8, u8, u8); 10] = [
    (220, 50,  50),   // red
    (50,  200, 50),   // green
    (50,  100, 220),  // blue
    (220, 180, 40),   // yellow
    (200, 50,  200),  // magenta
    (40,  200, 200),  // cyan
    (255, 140, 0),    // orange
    (150, 50,  220),  // purple
    (200, 200, 200),  // light grey
    (50,  200, 150),  // teal
];

fn palette_color(idx: usize) -> ConfigColor {
    let (r, g, b) = COLOR_PALETTE[idx % COLOR_PALETTE.len()];
    ConfigColor { r, g, b }
}

// --- selectable list helper ---------------------------------------------------

#[derive(Clone)]
struct SelectableList {
    items: Vec<[String; 2]>, // [topic_name, topic_type]
    state: ListState,
}

impl SelectableList {
    fn new(items: Vec<[String; 2]>) -> Self {
        SelectableList { items, state: ListState::default() }
    }

    fn next(&mut self) {
        if self.items.is_empty() { return; }
        let i = match self.state.selected() {
            Some(i) => if i + 1 >= self.items.len() { 0 } else { i + 1 },
            None => 0,
        };
        self.state.select(Some(i));
    }

    fn previous(&mut self) {
        if self.items.is_empty() { return; }
        let i = match self.state.selected() {
            Some(i) => if i == 0 { self.items.len() - 1 } else { i - 1 },
            None => 0,
        };
        self.state.select(Some(i));
    }

    /// Push an element to the end.
    fn push(&mut self, el: [String; 2]) {
        self.items.push(el);
    }

    /// Remove and return the currently selected element (or first).
    fn remove_selected(&mut self) -> Option<[String; 2]> {
        if self.items.is_empty() { return None; }
        let i = self.state.selected().unwrap_or(0).min(self.items.len() - 1);
        let el = self.items.remove(i);
        // keep selection in bounds
        if !self.items.is_empty() {
            self.state.select(Some(i.min(self.items.len() - 1)));
        } else {
            self.state.select(None);
        }
        Some(el)
    }

    fn ensure_selection(&mut self) {
        if self.state.selected().is_none() && !self.items.is_empty() {
            self.state.select(Some(0));
        }
    }
}

// --- TopicManager mode --------------------------------------------------------

enum SaveState {
    Unsaved,
    Saved,
    Error(String),
}

pub struct TopicManager {
    available: SelectableList, // supported, currently inactive topics
    active: SelectableList,    // topics that are in the current config
    config: Termviz2Config,    // original config (non-topic fields preserved on save)
    /// true = left panel focused; false = right panel focused
    left_focused: bool,
    save_state: SaveState,
    /// Running count of new topics added (used to pick palette colors).
    added_count: usize,
}

impl TopicManager {
    pub fn new(ros: Arc<ROS>, config: Termviz2Config) -> Self {
        // Collect all topics that are currently active in config.
        let active_topics = collect_active_topics(&config);

        // Query live topics from ROS2; fall back to empty list on error.
        let live_topics: Vec<[String; 2]> = match ros.node.get_topic_names_and_types() {
            Ok(map) => {
                let mut v: Vec<[String; 2]> = map
                    .into_iter()
                    .flat_map(|(name, types)| {
                        types.into_iter().map(move |t| [name.clone(), t])
                    })
                    .filter(|el| is_supported_type(&el[1]))
                    .filter(|el| !active_topics.contains(el))
                    .collect();
                v.sort();
                v
            }
            Err(e) => {
                eprintln!("TopicManager: failed to query ROS2 topics: {:?}", e);
                vec![]
            }
        };

        let mut available = SelectableList::new(live_topics);
        available.ensure_selection();

        TopicManager {
            available,
            active: SelectableList::new(active_topics),
            config,
            left_focused: true,
            save_state: SaveState::Unsaved,
            added_count: 0,
        }
    }

    fn focused_list_mut(&mut self) -> &mut SelectableList {
        if self.left_focused { &mut self.available } else { &mut self.active }
    }

    /// Move selected item from available → active.
    fn add_topic(&mut self) {
        if let Some(el) = self.available.remove_selected() {
            self.active.push(el);
            self.added_count += 1;
        }
    }

    /// Move selected item from active → available.
    fn remove_topic(&mut self) {
        if let Some(el) = self.active.remove_selected() {
            self.available.push(el);
            self.available.items.sort();
        }
    }

    fn switch_focus_right(&mut self) {
        self.left_focused = false;
        self.active.ensure_selection();
        self.available.state.select(None);
    }

    fn switch_focus_left(&mut self) {
        self.left_focused = true;
        self.available.ensure_selection();
        self.active.state.select(None);
    }

    fn save(&mut self) {
        let mut cfg = self.config.clone();

        // Clear all topic lists — will repopulate from self.active.
        cfg.laser_topics.clear();
        cfg.pointcloud2_topics.clear();
        cfg.image_topics.clear();
        cfg.pose_topics.clear();
        cfg.pose_array_topics.clear();
        cfg.path_topics.clear();
        cfg.map_topics.clear();
        cfg.marker_topics.clear();
        cfg.marker_array_topics.clear();
        cfg.polygon_topics.clear();

        let mut color_idx = 0usize;
        for topic in &self.active.items {
            let name = topic[0].clone();
            let ty = topic[1].as_str();

            match ty {
                "sensor_msgs/msg/LaserScan" => {
                    let existing_color = self.config.laser_topics.iter()
                        .find(|t| t.topic == name).map(|t| t.color.clone());
                    let color = existing_color.unwrap_or_else(|| { let c = palette_color(color_idx); color_idx += 1; c });
                    cfg.laser_topics.push(ListenerConfigColor { topic: name, color });
                }
                "sensor_msgs/msg/PointCloud2" => {
                    let existing = self.config.pointcloud2_topics.iter()
                        .find(|t| t.topic == name).cloned();
                    cfg.pointcloud2_topics.push(existing.unwrap_or(PointCloud2ListenerConfig {
                        topic: name,
                        use_rgb: false,
                        default_color: { let c = palette_color(color_idx); color_idx += 1; c },
                    }));
                }
                "sensor_msgs/msg/Image" => {
                    let existing = self.config.image_topics.iter()
                        .find(|t| t.topic == name).cloned();
                    cfg.image_topics.push(existing.unwrap_or(ImageListenerConfig {
                        topic: name,
                        rotation: 0,
                    }));
                }
                "geometry_msgs/msg/PoseStamped" => {
                    let existing = self.config.pose_topics.iter()
                        .find(|t| t.topic == name).cloned();
                    cfg.pose_topics.push(existing.unwrap_or_else(|| PoseListenerConfig {
                        topic: name,
                        color: { let c = palette_color(color_idx); color_idx += 1; c },
                        style: crate::config::PoseStyle::Arrow,
                        length: 0.2,
                    }));
                }
                "geometry_msgs/msg/PoseArray" => {
                    let existing = self.config.pose_array_topics.iter()
                        .find(|t| t.topic == name).cloned();
                    cfg.pose_array_topics.push(existing.unwrap_or_else(|| PoseListenerConfig {
                        topic: name,
                        color: { let c = palette_color(color_idx); color_idx += 1; c },
                        style: crate::config::PoseStyle::Arrow,
                        length: 0.2,
                    }));
                }
                "nav_msgs/msg/Path" => {
                    let existing = self.config.path_topics.iter()
                        .find(|t| t.topic == name).cloned();
                    cfg.path_topics.push(existing.unwrap_or_else(|| PathListenerConfig {
                        topic: name,
                        color: { let c = palette_color(color_idx); color_idx += 1; c },
                        style: crate::config::PoseStyle::Line,
                        length: 0.2,
                    }));
                }
                "nav_msgs/msg/OccupancyGrid" => {
                    let existing = self.config.map_topics.iter()
                        .find(|t| t.topic == name).cloned();
                    cfg.map_topics.push(existing.unwrap_or(MapListenerConfig {
                        topic: name,
                        color: ConfigColor { r: 255, g: 255, b: 255 },
                        threshold: 1,
                        transient_local: true,
                    }));
                }
                "visualization_msgs/msg/Marker" => {
                    cfg.marker_topics.push(ListenerConfig { topic: name });
                }
                "visualization_msgs/msg/MarkerArray" => {
                    cfg.marker_array_topics.push(ListenerConfig { topic: name });
                }
                "geometry_msgs/msg/PolygonStamped" => {
                    let existing_color = self.config.polygon_topics.iter()
                        .find(|t| t.topic == name).map(|t| t.color.clone());
                    let color = existing_color.unwrap_or_else(|| { let c = palette_color(color_idx); color_idx += 1; c });
                    cfg.polygon_topics.push(ListenerConfigColor { topic: name, color });
                }
                _ => {}
            }
        }

        // Save to user config (~/.config/termviz2/termviz2.yml), not system config.
        match confy::store("termviz2", "termviz2", &cfg) {
            Ok(_) => self.save_state = SaveState::Saved,
            Err(e) => self.save_state = SaveState::Error(format!("{:?}", e)),
        }
    }
}

// --- helpers ------------------------------------------------------------------

fn is_supported_type(ty: &str) -> bool {
    SUPPORTED_TYPES.iter().any(|(t, _)| *t == ty)
}

fn type_label(ty: &str) -> &str {
    SUPPORTED_TYPES.iter().find(|(t, _)| *t == ty).map(|(_, l)| *l).unwrap_or(ty)
}

fn collect_active_topics(cfg: &Termviz2Config) -> Vec<[String; 2]> {
    let mut v: Vec<[String; 2]> = Vec::new();
    for t in &cfg.laser_topics       { v.push([t.topic.clone(), "sensor_msgs/msg/LaserScan".into()]); }
    for t in &cfg.pointcloud2_topics { v.push([t.topic.clone(), "sensor_msgs/msg/PointCloud2".into()]); }
    for t in &cfg.image_topics       { v.push([t.topic.clone(), "sensor_msgs/msg/Image".into()]); }
    for t in &cfg.pose_topics        { v.push([t.topic.clone(), "geometry_msgs/msg/PoseStamped".into()]); }
    for t in &cfg.pose_array_topics  { v.push([t.topic.clone(), "geometry_msgs/msg/PoseArray".into()]); }
    for t in &cfg.path_topics        { v.push([t.topic.clone(), "nav_msgs/msg/Path".into()]); }
    for t in &cfg.map_topics         { v.push([t.topic.clone(), "nav_msgs/msg/OccupancyGrid".into()]); }
    for t in &cfg.marker_topics      { v.push([t.topic.clone(), "visualization_msgs/msg/Marker".into()]); }
    for t in &cfg.marker_array_topics{ v.push([t.topic.clone(), "visualization_msgs/msg/MarkerArray".into()]); }
    for t in &cfg.polygon_topics     { v.push([t.topic.clone(), "geometry_msgs/msg/PolygonStamped".into()]); }
    v
}

// --- trait impls --------------------------------------------------------------

impl BaseMode for TopicManager {}

impl AppMode for TopicManager {
    fn run(&mut self) {}
    fn reset(&mut self) {}

    fn get_name(&self) -> String {
        "Topic Manager".to_string()
    }

    fn get_description(&self) -> Vec<String> {
        vec![
            "Enable or disable topics. Changes take effect after restart.".to_string(),
            "Saves to ~/.config/termviz2/termviz2.yml (user config only).".to_string(),
        ]
    }

    fn get_keymap(&self) -> Vec<[String; 2]> {
        vec![
            [input::UP.to_string(), "Move selection up in focused panel".to_string()],
            [input::DOWN.to_string(), "Move selection down in focused panel".to_string()],
            [input::RIGHT.to_string(), "Add selected topic to active list".to_string()],
            [input::LEFT.to_string(), "Remove selected topic from active list".to_string()],
            [input::NEXT.to_string(), "Switch focus to Active Topics panel".to_string()],
            [input::PREVIOUS.to_string(), "Switch focus to Available Topics panel".to_string()],
            [input::CONFIRM.to_string(), "Save config".to_string()],
        ]
    }

    fn handle_input(&mut self, action: &str) {
        // Once saved, only allow switching focus (not editing).
        match action {
            input::UP   => { self.focused_list_mut().previous(); }
            input::DOWN => { self.focused_list_mut().next(); }
            input::RIGHT if self.left_focused  => self.add_topic(),
            input::LEFT  if !self.left_focused => self.remove_topic(),
            input::NEXT     => self.switch_focus_right(),
            input::PREVIOUS => self.switch_focus_left(),
            input::CONFIRM  => self.save(),
            _ => {}
        }
    }
}

impl Drawable for TopicManager {
    fn draw(&self, f: &mut Frame) {
        let area = f.area();

        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .horizontal_margin(4)
            .vertical_margin(1)
            .constraints([
                Constraint::Length(3), // title
                Constraint::Length(2), // status / hint bar
                Constraint::Min(4),    // two topic lists
            ])
            .split(area);

        // Title
        let title = Paragraph::new(Line::from(Span::styled(
            "Topic Manager",
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
        )))
        .block(Block::default().borders(Borders::ALL))
        .alignment(Alignment::Center);
        f.render_widget(title, chunks[0]);

        // Status / hint bar
        let status_text = match &self.save_state {
            SaveState::Saved => Line::from(vec![
                Span::styled("✓ Saved. ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
                Span::raw("Restart termviz2 to apply changes."),
            ]),
            SaveState::Error(e) => Line::from(Span::styled(
                format!("✗ Save failed: {}", e),
                Style::default().fg(Color::Red),
            )),
            SaveState::Unsaved => Line::from(vec![
                Span::styled(
                    if self.left_focused { "[Available] " } else { "[Active] " },
                    Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
                ),
                Span::raw("w/s: navigate  "),
                if self.left_focused {
                    Span::styled("d: add  ", Style::default().fg(Color::Green))
                } else {
                    Span::styled("a: remove  ", Style::default().fg(Color::Red))
                },
                Span::raw("n/b: switch panel  Enter: save"),
            ]),
        };
        let hint = Paragraph::new(status_text)
            .alignment(Alignment::Center)
            .wrap(Wrap { trim: false });
        f.render_widget(hint, chunks[1]);

        // Two-column layout for the topic lists
        let cols = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(chunks[2]);

        // Border color: highlight focused panel
        let avail_border_style = if self.left_focused {
            Style::default().fg(Color::Yellow)
        } else {
            Style::default().fg(Color::DarkGray)
        };
        let active_border_style = if !self.left_focused {
            Style::default().fg(Color::Yellow)
        } else {
            Style::default().fg(Color::DarkGray)
        };

        // Available topics list
        let avail_items: Vec<ListItem> = self
            .available
            .items
            .iter()
            .map(|i| {
                let label = type_label(&i[1]);
                ListItem::new(Line::from(vec![
                    Span::raw(i[0].clone()),
                    Span::styled(
                        format!("  [{}]", label),
                        Style::default().fg(Color::DarkGray),
                    ),
                ]))
            })
            .collect();
        let avail_list = List::new(avail_items)
            .block(
                Block::default()
                    .title(format!(" Available Topics ({}) ", self.available.items.len()))
                    .borders(Borders::ALL)
                    .border_style(avail_border_style),
            )
            .highlight_style(
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            )
            .highlight_symbol("▶ ");

        // Active topics list
        let active_items: Vec<ListItem> = self
            .active
            .items
            .iter()
            .map(|i| {
                let label = type_label(&i[1]);
                ListItem::new(Line::from(vec![
                    Span::raw(i[0].clone()),
                    Span::styled(
                        format!("  [{}]", label),
                        Style::default().fg(Color::DarkGray),
                    ),
                ]))
            })
            .collect();
        let active_list = List::new(active_items)
            .block(
                Block::default()
                    .title(format!(" Active Topics ({}) ", self.active.items.len()))
                    .borders(Borders::ALL)
                    .border_style(active_border_style),
            )
            .highlight_style(
                Style::default()
                    .fg(Color::Green)
                    .add_modifier(Modifier::BOLD),
            )
            .highlight_symbol("▶ ");

        f.render_stateful_widget(avail_list, cols[0], &mut self.available.state.clone());
        f.render_stateful_widget(active_list, cols[1], &mut self.active.state.clone());
    }
}
