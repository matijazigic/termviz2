use crate::config::ImageListenerConfig;
use crate::ros::ROS;
use byteorder::{ByteOrder, LittleEndian};
use image::{DynamicImage, ImageBuffer, RgbImage};
use rclrs::{RclrsError, SubscriptionOptions};
use std::sync::atomic::{AtomicI64, AtomicU64, Ordering};
use std::sync::{Arc, RwLock};

fn remap_u8(val: f64, min_val: f64, max_val: f64) -> u8 {
    if (max_val - min_val).abs() < f64::EPSILON {
        return 0;
    }
    ((val - min_val) * (u8::MAX as f64 / (max_val - min_val))).clamp(0.0, 255.0) as u8
}

fn decode_image(msg: &sensor_msgs::msg::Image) -> Option<DynamicImage> {
    let w = msg.width;
    let h = msg.height;
    match msg.encoding.as_str() {
        "8UC1" | "mono8" => {
            let buf = ImageBuffer::from_raw(w, h, msg.data.clone())?;
            Some(DynamicImage::ImageLuma8(buf))
        }
        "rgb8" | "8UC3" => {
            let buf: RgbImage = ImageBuffer::from_raw(w, h, msg.data.clone())?;
            Some(DynamicImage::ImageRgb8(buf))
        }
        "bgr8" => {
            let buf: RgbImage = ImageBuffer::from_fn(w, h, |x, y| {
                let idx = ((y * msg.step + x * 3) as usize).min(msg.data.len().saturating_sub(3));
                image::Rgb([msg.data[idx + 2], msg.data[idx + 1], msg.data[idx]])
            });
            Some(DynamicImage::ImageRgb8(buf))
        }
        "16UC1" | "mono16" => {
            let vals: Vec<u16> = msg.data.chunks(2).map(|c| LittleEndian::read_u16(c)).collect();
            let max_v = *vals.iter().max().unwrap_or(&1) as f64;
            let min_v = *vals.iter().min().unwrap_or(&0) as f64;
            let bytes: Vec<u8> = vals.iter().map(|&v| remap_u8(v as f64, min_v, max_v)).collect();
            let buf = ImageBuffer::from_raw(w, h, bytes)?;
            Some(DynamicImage::ImageLuma8(buf))
        }
        "32FC1" => {
            let floats: Vec<f32> = msg.data.chunks(4).map(|c| LittleEndian::read_f32(c)).collect();
            let max_v = floats.iter().cloned().fold(f32::NEG_INFINITY, f32::max) as f64;
            let min_v = floats.iter().cloned().fold(f32::INFINITY, f32::min) as f64;
            let bytes: Vec<u8> = floats.iter().map(|&v| remap_u8(v as f64, min_v, max_v)).collect();
            let buf = ImageBuffer::from_raw(w, h, bytes)?;
            Some(DynamicImage::ImageLuma8(buf))
        }
        enc => {
            eprintln!("Image encoding '{}' not supported", enc);
            None
        }
    }
}

fn apply_rotation(img: DynamicImage, rotation: i64) -> DynamicImage {
    match ((rotation % 360) + 360) % 360 {
        90  => DynamicImage::ImageRgba8(image::imageops::rotate90(&img.to_rgba8())),
        180 => DynamicImage::ImageRgba8(image::imageops::rotate180(&img.to_rgba8())),
        270 => DynamicImage::ImageRgba8(image::imageops::rotate270(&img.to_rgba8())),
        _   => img,
    }
}

pub struct ImageListener {
    pub config: ImageListenerConfig,
    pub img: Arc<RwLock<Option<DynamicImage>>>,
    pub frame_id: Arc<AtomicU64>,
    pub rotation: Arc<AtomicI64>,
    ros: Arc<ROS>,
    _sub: Option<Arc<dyn std::any::Any + Send + Sync>>,
}

impl ImageListener {
    /// Create a listener without subscribing. Call `activate()` to start receiving.
    pub fn new(config: ImageListenerConfig, ros: Arc<ROS>) -> Self {
        let initial_rotation = config.rotation;
        ImageListener {
            config,
            img: Arc::new(RwLock::new(None)),
            frame_id: Arc::new(AtomicU64::new(0)),
            rotation: Arc::new(AtomicI64::new(initial_rotation)),
            ros,
            _sub: None,
        }
    }

    /// Rotate by `delta` degrees (±90). Wraps around 0/90/180/270.
    pub fn rotate(&self, delta: i64) {
        let prev = self.rotation.load(Ordering::Relaxed);
        let next = ((prev + delta) % 360 + 360) % 360;
        self.rotation.store(next, Ordering::Relaxed);
    }

    pub fn is_active(&self) -> bool {
        self._sub.is_some()
    }

    pub fn activate(&mut self) -> Result<(), RclrsError> {
        if self._sub.is_some() {
            return Ok(());
        }
        let img_cb = Arc::clone(&self.img);
        let frame_id_cb = Arc::clone(&self.frame_id);
        let rotation_cb = Arc::clone(&self.rotation);

        let sub_options = SubscriptionOptions::new(&self.config.topic);
        let sub = self.ros.node.create_subscription::<sensor_msgs::msg::Image, _>(
            sub_options,
            move |msg: sensor_msgs::msg::Image| {
                if let Some(decoded) = decode_image(&msg) {
                    let rotation = rotation_cb.load(Ordering::Relaxed);
                    let rotated = apply_rotation(decoded, rotation);
                    // try_write: skip frame if the render thread is currently reading.
                    if let Ok(mut guard) = img_cb.try_write() {
                        *guard = Some(rotated);
                        frame_id_cb.fetch_add(1, Ordering::Relaxed);
                    }
                }
            },
        )?;
        self._sub = Some(Arc::new(sub));
        Ok(())
    }

    pub fn deactivate(&mut self) {
        self._sub = None;
        // Don't block waiting for the lock — the image will be overwritten on
        // next activation anyway. Drop subscription first so no new frames arrive.
        if let Ok(mut guard) = self.img.try_write() {
            *guard = None;
        }
    }
}
