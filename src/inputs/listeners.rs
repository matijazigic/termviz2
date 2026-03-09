use crate::config::Termviz2Config;
use crate::inputs::polygon::PolygonStampedListener;
use crate::inputs::laser::LaserListener;
use crate::inputs::map::MapListener;
use crate::inputs::path::PathListener;
use crate::inputs::pointcloud::PointCloud2Listener;
use crate::inputs::pose::{PoseArrayListener, PoseStampedListener};
use crate::ros::ROS;
use std::sync::Arc;

pub struct Listeners {
    pub maps: Vec<MapListener>,
    pub polygons: Vec<PolygonStampedListener>,
    pub lasers: Vec<LaserListener>,
    pub poses: Vec<PoseStampedListener>,
    pub pose_arrays: Vec<PoseArrayListener>,
    pub paths: Vec<PathListener>,
    pub pointclouds: Vec<PointCloud2Listener>,
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

        let polygons = conf.polygon_topics.iter().filter_map(|cfg| {
            match PolygonStampedListener::new(cfg.clone(), Arc::clone(&ros), conf.fixed_frame.clone()) {
                Ok(l) => Some(l),
                Err(e) => {
                    eprintln!("Failed to create polygon listener for '{}': {:?}", cfg.topic, e);
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

        let poses = conf.pose_topics.iter().filter_map(|cfg| {
            match PoseStampedListener::new(cfg.clone(), Arc::clone(&ros), conf.fixed_frame.clone()) {
                Ok(l) => Some(l),
                Err(e) => {
                    eprintln!("Failed to create pose listener for '{}': {:?}", cfg.topic, e);
                    None
                }
            }
        }).collect();

        let pose_arrays = conf.pose_array_topics.iter().filter_map(|cfg| {
            match PoseArrayListener::new(cfg.clone(), Arc::clone(&ros), conf.fixed_frame.clone()) {
                Ok(l) => Some(l),
                Err(e) => {
                    eprintln!("Failed to create pose array listener for '{}': {:?}", cfg.topic, e);
                    None
                }
            }
        }).collect();

        let paths = conf.path_topics.iter().filter_map(|cfg| {
            match PathListener::new(cfg.clone(), Arc::clone(&ros), conf.fixed_frame.clone()) {
                Ok(l) => Some(l),
                Err(e) => {
                    eprintln!("Failed to create path listener for '{}': {:?}", cfg.topic, e);
                    None
                }
            }
        }).collect();

        let pointclouds = conf.pointcloud2_topics.iter().filter_map(|cfg| {
            match PointCloud2Listener::new(cfg.clone(), Arc::clone(&ros), conf.fixed_frame.clone()) {
                Ok(l) => Some(l),
                Err(e) => {
                    eprintln!("Failed to create pointcloud2 listener for '{}': {:?}", cfg.topic, e);
                    None
                }
            }
        }).collect();

        Listeners { maps, polygons, lasers, poses, pose_arrays, paths, pointclouds }
    }
}
