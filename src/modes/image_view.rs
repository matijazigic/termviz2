use crate::inputs::image::ImageListener;
use crate::modes::{AppMode, BaseMode, Drawable};
use image::DynamicImage;
use ratatui::layout::{Alignment, Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line as TextLine, Span, Text};
use ratatui::widgets::{Block, Borders, Paragraph, Wrap};
use ratatui::Frame;
use std::sync::Mutex;
use std::sync::atomic::Ordering;

/// Cached render output, keyed by (frame_id, area). Avoids re-scaling on every
/// draw tick when the image and terminal size have not changed.
struct RenderCache {
    frame_id: u64,
    area: Rect,
    lines: Vec<TextLine<'static>>,
}

pub struct ImageView {
    pub images: Vec<ImageListener>,
    active_sub: usize,
    cache: Mutex<Option<RenderCache>>,
}

impl ImageView {
    pub fn new(images: Vec<ImageListener>) -> Self {
        ImageView { images, active_sub: 0, cache: Mutex::new(None) }
    }
}

/// Scale `img` to fit within `area` (preserving aspect ratio) using a CatmullRom
/// filter, then convert to half-block spans.
fn build_halfblock_lines(img: &DynamicImage, area: Rect) -> Option<Vec<TextLine<'static>>> {
    if area.width == 0 || area.height == 0 {
        return None;
    }

    let target_w = area.width as u32;
    let target_h = area.height as u32 * 2; // each cell = 2 pixel rows

    // Pre-scale with quality filter, preserving aspect ratio.
    let scaled = img.resize(target_w, target_h, image::imageops::FilterType::CatmullRom);
    let rgb = scaled.to_rgb8();
    let (sw, sh) = rgb.dimensions();

    let cell_rows = (sh + 1) / 2;
    let mut lines: Vec<TextLine<'static>> = Vec::with_capacity(cell_rows as usize);

    for row in 0..cell_rows {
        let py0 = row * 2;
        let py1 = row * 2 + 1;
        let mut spans: Vec<Span<'static>> = Vec::with_capacity(sw as usize);
        for col in 0..sw {
            let [r0, g0, b0] = rgb.get_pixel(col, py0).0;
            let (r1, g1, b1) = if py1 < sh {
                let p = rgb.get_pixel(col, py1).0;
                (p[0], p[1], p[2])
            } else {
                (0, 0, 0)
            };
            spans.push(Span::styled(
                "▀",
                Style::default()
                    .fg(Color::Rgb(r0, g0, b0))
                    .bg(Color::Rgb(r1, g1, b1)),
            ));
        }
        lines.push(TextLine::from(spans));
    }

    Some(lines)
}

impl AppMode for ImageView {
    fn run(&mut self) {
        if self.images.is_empty() {
            return;
        }
        if !self.images[self.active_sub].is_active() {
            if let Err(e) = self.images[self.active_sub].activate() {
                eprintln!("Failed to activate image listener for '{}': {:?}",
                    self.images[self.active_sub].config.topic, e);
            }
        }
    }

    fn reset(&mut self) {
        for img in &mut self.images {
            img.deactivate();
        }
        *self.cache.lock().unwrap() = None;
    }

    fn handle_input(&mut self, action: &str) {
        use crate::modes::input;
        if self.images.is_empty() {
            return;
        }
        match action {
            input::LEFT | input::PREVIOUS => {
                self.images[self.active_sub].deactivate();
                self.active_sub = if self.active_sub > 0 {
                    self.active_sub - 1
                } else {
                    self.images.len() - 1
                };
                *self.cache.lock().unwrap() = None;
            }
            input::RIGHT | input::NEXT => {
                self.images[self.active_sub].deactivate();
                self.active_sub = (self.active_sub + 1) % self.images.len();
                *self.cache.lock().unwrap() = None;
            }
            input::ROTATE_LEFT => {
                self.images[self.active_sub].rotate(-90);
                *self.cache.lock().unwrap() = None;
            }
            input::ROTATE_RIGHT => {
                self.images[self.active_sub].rotate(90);
                *self.cache.lock().unwrap() = None;
            }
            _ => {}
        }
    }

    fn get_name(&self) -> String {
        "Image".to_string()
    }

    fn get_description(&self) -> Vec<String> {
        vec!["Visualizes images received on the configured topics.".to_string()]
    }

    fn get_keymap(&self) -> Vec<[String; 2]> {
        use crate::modes::input;
        vec![
            [input::LEFT.to_string(), "Previous image topic".to_string()],
            [input::RIGHT.to_string(), "Next image topic".to_string()],
            [input::ROTATE_LEFT.to_string(), "Rotate image 90° counter-clockwise".to_string()],
            [input::ROTATE_RIGHT.to_string(), "Rotate image 90° clockwise".to_string()],
        ]
    }
}

impl Drawable for ImageView {
    fn draw(&self, f: &mut Frame) {
        let area = f.area();
        let chunks = Layout::default()
            .constraints([Constraint::Length(1), Constraint::Percentage(100)])
            .split(area);

        if self.images.is_empty() {
            let header = Paragraph::new("Image view — no topic configured!")
                .block(Block::default().borders(Borders::NONE))
                .style(Style::default().fg(Color::White))
                .alignment(Alignment::Center)
                .wrap(Wrap { trim: false });
            f.render_widget(header, chunks[0]);
            return;
        }

        let active = &self.images[self.active_sub];
        let current_frame_id = active.frame_id.load(Ordering::Relaxed);
        let img_guard = active.img.read().unwrap();

        let dim_str = if let Some(img) = img_guard.as_ref() {
            format!(" {}×{}", img.width(), img.height())
        } else {
            String::new()
        };

        let header = Paragraph::new(TextLine::from(vec![
            Span::styled(
                "Image view",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::raw(format!(
                " — {} ({}/{}){}",
                active.config.topic,
                self.active_sub + 1,
                self.images.len(),
                dim_str,
            )),
        ]))
        .block(Block::default().borders(Borders::NONE))
        .style(Style::default().fg(Color::White))
        .alignment(Alignment::Left);
        f.render_widget(header, chunks[0]);

        let content_area = chunks[1];

        if let Some(img) = img_guard.as_ref() {
            let mut cache = self.cache.lock().unwrap();

            let cache_hit = cache.as_ref().map_or(false, |c| {
                c.frame_id == current_frame_id && c.area == content_area
            });

            if !cache_hit {
                if let Some(lines) = build_halfblock_lines(img, content_area) {
                    *cache = Some(RenderCache {
                        frame_id: current_frame_id,
                        area: content_area,
                        lines,
                    });
                }
            }

            if let Some(cached) = cache.as_ref() {
                let paragraph = Paragraph::new(Text::from(cached.lines.clone()));
                f.render_widget(paragraph, content_area);
            }
        } else {
            let waiting = Paragraph::new("Waiting for image…")
                .style(Style::default().fg(Color::DarkGray))
                .alignment(Alignment::Center);
            f.render_widget(waiting, content_area);
        }
    }
}

impl BaseMode for ImageView {}
