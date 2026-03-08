use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;
use ratatui::style::Color as TuiColor;

fn default_map_color() -> Color {
    Color { r: 255, g: 255, b: 255 }
}

fn default_map_threshold() -> i8 {
    1
}

const fn default_int() -> i64 {
    0
}

const fn color_white() -> Color {
    Color {
        r: 255,
        g: 255,
        b: 255,
    }
}

const fn color_red() -> Color {
    Color { r: 255, g: 0, b: 0 }
}

const fn color_blue() -> Color {
    Color { r: 0, g: 0, b: 255 }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Color {
    pub r: u8,
    pub b: u8,
    pub g: u8,
}

impl Color {
    pub fn to_tui(&self) -> TuiColor {
        return TuiColor::Rgb(self.r, self.g, self.b);
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ListenerConfig {
    pub topic: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ListenerConfigColor {
    pub topic: String,
    pub color: Color,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct MapListenerConfig {
    pub topic: String,
    #[serde(default = "default_map_color")]
    pub color: Color,
    #[serde(default = "default_map_threshold")]
    pub threshold: i8,
}


const DEFAULT_PATH: &str = "/etc/termviz2/termviz2.yml";

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Termviz2Config {
    pub fixed_frame: String,
    pub robot_frame: String,
    pub tf_topic: String,
    pub tf_static_topic: String,
    pub target_framerate: u64,
    pub axis_length: f64,
    pub visible_area: Vec<f64>, // [x_min, x_max, y_min, y_max] in meters from center
    pub zoom_factor: f64,
    pub key_mapping: HashMap<String, String>,
    #[serde(default)]
    pub map_topics: Vec<MapListenerConfig>,
    #[serde(default)]
    pub footprint_topics: Vec<ListenerConfigColor>,
    #[serde(default)]
    pub laser_topics: Vec<ListenerConfigColor>,
}

impl Default for Termviz2Config {
    fn default() -> Self {
        Termviz2Config {
            fixed_frame: "map".to_string(),
            robot_frame: "base_link".to_string(),
            tf_topic: "/tf".to_string(),
            tf_static_topic: "/tf_static".to_string(),
            target_framerate: 30,
            axis_length: 0.5,
            visible_area: vec![-5.0, 5.0, -5.0, 5.0],
            zoom_factor: 0.1,
            key_mapping: HashMap::from([
                ("up".to_string(), "w".to_string()),
                ("down".to_string(), "s".to_string()),
                ("left".to_string(), "a".to_string()),
                ("right".to_string(), "d".to_string()),
                ("zoom_in".to_string(), "=".to_string()),
                ("zoom_out".to_string(), "-".to_string()),
                ("next".to_string(), "n".to_string()),
                ("previous".to_string(), "b".to_string()),
                ("show_help".to_string(), "h".to_string()),
            ]),
            map_topics: vec![MapListenerConfig {
                topic: "map".to_string(),
                color: default_map_color(),
                threshold: default_map_threshold(),
            }],
            footprint_topics: vec![ListenerConfigColor {
                topic: "/robot_0/local_costmap/published_footprint".to_string(),
                color: color_blue(),
            }],
            laser_topics: vec![ListenerConfigColor {
                topic: "scan".to_string(),
                color: Color { r: 200, g: 0, b: 0 },
            }],
        }
    }
}

/// Loads config with the following priority:
/// 1. Custom path, if provided via CLI
/// 2. User config path (~/.config/termviz2/termviz2.yml), if it exists
/// 3. Global default path (/etc/termviz2/termviz2.yml), if it exists
/// 4. Built-in defaults
pub fn get_config(custom_path: Option<&PathBuf>) -> Result<Termviz2Config, confy::ConfyError> {
    let default_path = PathBuf::from(DEFAULT_PATH);
    let user_path = confy::get_configuration_file_path("termviz2", "termviz2")?;

    let load_config_path: &PathBuf = custom_path.unwrap_or_else(|| {
        if user_path.exists() {
            &user_path
        } else {
            &default_path
        }
    });

    let cfg: Termviz2Config = if load_config_path.exists() {
        println!("Loading config from: {:?}", load_config_path);
        confy::load_path(load_config_path)?
    } else {
        println!("No config found at {:?}, using defaults.", load_config_path);
        Termviz2Config::default()
    };

    if custom_path.is_none() {
        match confy::store_path(user_path.as_path(), &cfg) {
            Ok(_) => println!("Updated {:?}", user_path),
            Err(e) => println!("Error updating config at user path: {:?}", e),
        }
    }

    Ok(cfg)
}
