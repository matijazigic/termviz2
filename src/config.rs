use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;

fn default_map_color() -> Color {
    Color { r: 255, g: 255, b: 255 }
}

fn default_map_threshold() -> i8 {
    1
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


#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ListenerConfig {
    pub topic: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ListenerConfigColor {
    pub topic: String,
    pub color: Color,
}

#[derive(Debug, Serialize, Deserialize, Clone, Default, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum PoseStyle {
    #[default]
    Arrow,
    Axes,
    Line,
}

fn default_pose_style() -> PoseStyle {
    PoseStyle::default()
}

const fn default_pose_length() -> f64 { 0.2 }
fn default_path_style() -> PoseStyle { PoseStyle::Line }

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PoseListenerConfig {
    pub topic: String,
    pub color: Color,
    #[serde(default = "default_pose_style")]
    pub style: PoseStyle,
    #[serde(default = "default_pose_length")]
    pub length: f64,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PathListenerConfig {
    pub topic: String,
    pub color: Color,
    #[serde(default = "default_path_style")]
    pub style: PoseStyle,
    #[serde(default = "default_pose_length")]
    pub length: f64,
}

fn default_pointcloud_color() -> Color {
    Color { r: 255, g: 255, b: 255 }
}


#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PointCloud2ListenerConfig {
    pub topic: String,
    #[serde(default = "bool::default")]
    pub use_rgb: bool,
    #[serde(default = "default_pointcloud_color")]
    pub default_color: Color,
}

const fn default_true() -> bool { true }

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct MapListenerConfig {
    pub topic: String,
    #[serde(default = "default_map_color")]
    pub color: Color,
    #[serde(default = "default_map_threshold")]
    pub threshold: i8,
    /// true  = TransientLocal QoS (static map)
    /// false = Volatile QoS + RViz cost-colour gradient (costmap)
    #[serde(default = "default_true")]
    pub transient_local: bool,
}


#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ImageListenerConfig {
    pub topic: String,
    #[serde(default)]
    pub rotation: i64,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SendPoseConfig {
    pub topic: String,
    /// One of: "Pose", "PoseStamped", "PoseWithCovarianceStamped"
    pub msg_type: String,
    /// Optional polygon topic to use as ghost footprint (must be in polygon_topics).
    #[serde(default)]
    pub footprint_topic: Option<String>,
    /// Color of the ghost footprint. Defaults to gray if not set.
    #[serde(default = "default_ghost_footprint_color")]
    pub footprint_color: Color,
}

fn default_ghost_footprint_color() -> Color {
    Color { r: 128, g: 128, b: 128 }
}

#[derive(Debug, Serialize, Deserialize, Clone, Default, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum TeleopMode {
    Classic,
    #[default]
    Safe,
}

const fn default_teleop_increment() -> f64 { 0.1 }
const fn default_teleop_max_vel() -> f64 { 0.5 }

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct TeleopConfig {
    pub cmd_vel_topic: String,
    #[serde(default = "default_teleop_increment")]
    pub default_increment: f64,
    #[serde(default = "default_teleop_increment")]
    pub increment_step: f64,
    #[serde(default = "bool::default")]
    pub publish_cmd_vel_when_idle: bool,
    #[serde(default)]
    pub mode: TeleopMode,
    #[serde(default = "default_teleop_max_vel")]
    pub max_vel: f64,
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
    pub polygon_topics: Vec<ListenerConfigColor>,
    #[serde(default)]
    pub laser_topics: Vec<ListenerConfigColor>,
    #[serde(default)]
    pub pose_topics: Vec<PoseListenerConfig>,
    #[serde(default)]
    pub pose_array_topics: Vec<PoseListenerConfig>,
    #[serde(default)]
    pub path_topics: Vec<PathListenerConfig>,
    #[serde(default)]
    pub pointcloud2_topics: Vec<PointCloud2ListenerConfig>,
    #[serde(default)]
    pub marker_topics: Vec<ListenerConfig>,
    #[serde(default)]
    pub marker_array_topics: Vec<ListenerConfig>,
    #[serde(default)]
    pub image_topics: Vec<ImageListenerConfig>,
    #[serde(default)]
    pub send_pose_topics: Vec<SendPoseConfig>,
    #[serde(default)]
    pub teleop: Option<TeleopConfig>,
    #[serde(default)]
    pub odom_topics: Vec<ListenerConfig>,
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
                ("confirm".to_string(), "Enter".to_string()),
                ("cancel".to_string(), "Esc".to_string()),
                ("increment_step".to_string(), "e".to_string()),
                ("decrement_step".to_string(), "q".to_string()),
            ]),
            map_topics: vec![MapListenerConfig {
                topic: "map".to_string(),
                color: default_map_color(),
                threshold: default_map_threshold(),
                transient_local: true,
            }],
            polygon_topics: vec![ListenerConfigColor {
                topic: "/robot_0/local_costmap/published_footprint".to_string(),
                color: color_blue(),
            }],
            laser_topics: vec![ListenerConfigColor {
                topic: "scan".to_string(),
                color: Color { r: 200, g: 0, b: 0 },
            }],
            pose_topics: vec![],
            pose_array_topics: vec![],
            path_topics: vec![],
            pointcloud2_topics: vec![],
            marker_topics: vec![],
            marker_array_topics: vec![],
            image_topics: vec![],
            send_pose_topics: vec![],
            teleop: None,
            odom_topics: vec![],
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
