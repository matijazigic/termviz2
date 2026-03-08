use crate::config::Termviz2Config;
use crate::inputs::footprint::FootprintListener;
use crate::inputs::laser::LaserListener;
use crate::inputs::map::MapListener;
use crate::ros::ROS;
use std::sync::Arc;

pub struct Listeners {
    pub maps: Vec<MapListener>,
    pub footprints: Vec<FootprintListener>,
    pub lasers: Vec<LaserListener>,
}

impl Listeners {
    pub fn new(ros: Arc<ROS>, conf: &Termviz2Config) -> Self {
        let maps = conf.map_topics.iter().filter_map(|cfg| {
            match MapListener::new(cfg.clone(), Arc::clone(&ros), conf.fixed_frame.clone()) {
                Ok(l) => Some(l),
                Err(e) => {
                    eprintln!("Failed to create map listener for '{}': {:?}", cfg.topic, e);
                    None
                }
            }
        }).collect();

        let footprints = conf.footprint_topics.iter().filter_map(|cfg| {
            match FootprintListener::new(cfg.clone(), Arc::clone(&ros), conf.fixed_frame.clone()) {
                Ok(l) => Some(l),
                Err(e) => {
                    eprintln!("Failed to create footprint listener for '{}': {:?}", cfg.topic, e);
                    None
                }
            }
        }).collect();

        let lasers = conf.laser_topics.iter().filter_map(|cfg| {
            match LaserListener::new(cfg.clone(), Arc::clone(&ros), conf.fixed_frame.clone()) {
                Ok(l) => Some(l),
                Err(e) => {
                    eprintln!("Failed to create laser listener for '{}': {:?}", cfg.topic, e);
                    None
                }
            }
        }).collect();

        Listeners { maps, footprints, lasers }
    }
}
