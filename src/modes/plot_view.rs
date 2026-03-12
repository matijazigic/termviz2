//! Interactive plot mode — browse available ROS topics at runtime and plot fields.

use crate::inputs::plot::{fields_for_type, short_msg_type, PlotListener};
use crate::modes::{input, AppMode, BaseMode, Drawable};
use crate::ros::ROS;
use ratatui::Frame;
use ratatui::layout::{Alignment, Constraint, Direction, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::symbols;
use ratatui::text::{Line as TextLine, Span};
use ratatui::widgets::{Axis, Block, Borders, Chart, Dataset, List, ListItem, ListState, Paragraph};
use std::collections::VecDeque;
use std::sync::Arc;
use std::time::Instant;

const MAX_BUFFER: usize = 2000;

const PALETTE: &[Color] = &[
    Color::Red, Color::Green, Color::Yellow,
    Color::Cyan, Color::Magenta, Color::Blue,
    Color::LightRed, Color::LightGreen, Color::LightYellow,
];

// ── State machine ─────────────────────────────────────────────────────────────

struct BrowsingState {
    /// Supported topics: (topic_name, full_type_str, short_msg_type)
    topics: Vec<(String, String, &'static str)>,
    cursor: usize,
}

struct FieldState {
    topic: String,
    msg_type: &'static str,
    fields: &'static [&'static str],
    selected: Vec<bool>,
    cursor: usize,
}

struct PlottingState {
    listener: PlotListener,
    buffers: Vec<VecDeque<(f64, f64)>>,
    visible: Vec<bool>,
    field_cursor: usize,
    window_secs: f64,
    start: Instant,
}

enum State {
    Browsing(BrowsingState),
    SelectingFields(FieldState),
    Plotting(PlottingState),
}

// ── PlotView ──────────────────────────────────────────────────────────────────

pub struct PlotView {
    ros: Arc<ROS>,
    state: State,
}

impl PlotView {
    pub fn new(ros: Arc<ROS>) -> Self {
        let state = State::Browsing(Self::make_browse_state(&ros));
        PlotView { ros, state }
    }

    fn make_browse_state(ros: &Arc<ROS>) -> BrowsingState {
        let mut topics: Vec<(String, String, &'static str)> = ros
            .node
            .get_topic_names_and_types()
            .unwrap_or_default()
            .into_iter()
            .flat_map(|(name, types)| {
                types.into_iter().filter_map(move |t| {
                    short_msg_type(&t).map(|short| (name.clone(), t, short))
                })
            })
            .collect();
        topics.sort_by(|a, b| a.0.cmp(&b.0));
        BrowsingState { topics, cursor: 0 }
    }

    fn start_plotting(&mut self, fs: FieldState) {
        let selected_fields: Vec<String> = fs
            .fields
            .iter()
            .enumerate()
            .filter(|(i, _)| fs.selected[*i])
            .map(|(_, f)| f.to_string())
            .collect();

        if selected_fields.is_empty() {
            // nothing selected — go back to field picker
            self.state = State::SelectingFields(fs);
            return;
        }

        let n = selected_fields.len();
        // visible mirrors the selected_fields order (all visible initially)
        let visible = vec![true; n];
        let buffers = (0..n).map(|_| VecDeque::new()).collect();

        match PlotListener::new(&fs.topic, fs.msg_type, selected_fields, &self.ros) {
            Some(listener) => {
                self.state = State::Plotting(PlottingState {
                    listener,
                    buffers,
                    visible,
                    field_cursor: 0,
                    window_secs: 30.0,
                    start: Instant::now(),
                });
            }
            None => {
                // subscription failed — go back to browsing
                self.state = State::Browsing(Self::make_browse_state(&self.ros));
            }
        }
    }
}

// ── AppMode ───────────────────────────────────────────────────────────────────

impl AppMode for PlotView {
    fn run(&mut self) {
        if let State::Plotting(ps) = &mut self.state {
            let t = ps.start.elapsed().as_secs_f64();
            let vals = ps.listener.values.read().unwrap().clone();
            for (fi, &val) in vals.iter().enumerate() {
                if fi >= ps.buffers.len() { break; }
                let buf = &mut ps.buffers[fi];
                buf.push_back((t, val));
                while buf.len() > MAX_BUFFER { buf.pop_front(); }
            }
        }
    }

    fn reset(&mut self) {
        self.state = State::Browsing(Self::make_browse_state(&self.ros));
    }

    fn handle_input(&mut self, action: &str) {
        match &mut self.state {
            // ── Browsing ──────────────────────────────────────────────────────
            State::Browsing(bs) => {
                match action {
                    input::UP | input::PREVIOUS => {
                        if !bs.topics.is_empty() {
                            bs.cursor = if bs.cursor > 0 { bs.cursor - 1 } else { bs.topics.len() - 1 };
                        }
                    }
                    input::DOWN | input::NEXT => {
                        if !bs.topics.is_empty() {
                            bs.cursor = (bs.cursor + 1) % bs.topics.len();
                        }
                    }
                    input::CONFIRM => {
                        if let Some((topic, _type_str, msg_type)) = bs.topics.get(bs.cursor) {
                            let fields = fields_for_type(_type_str).unwrap_or(&[]);
                            let selected = vec![false; fields.len()];
                            let fs = FieldState {
                                topic: topic.clone(),
                                msg_type,
                                fields,
                                selected,
                                cursor: 0,
                            };
                            self.state = State::SelectingFields(fs);
                        }
                    }
                    // Refresh topic list
                    input::CANCEL => {
                        self.state = State::Browsing(Self::make_browse_state(&self.ros));
                    }
                    _ => {}
                }
            }

            // ── Selecting fields ──────────────────────────────────────────────
            State::SelectingFields(fs) => {
                match action {
                    input::UP | input::PREVIOUS => {
                        if !fs.fields.is_empty() {
                            fs.cursor = if fs.cursor > 0 { fs.cursor - 1 } else { fs.fields.len() - 1 };
                        }
                    }
                    input::DOWN | input::NEXT => {
                        if !fs.fields.is_empty() {
                            fs.cursor = (fs.cursor + 1) % fs.fields.len();
                        }
                    }
                    input::CONFIRM => {
                        if fs.cursor < fs.fields.len() {
                            fs.selected[fs.cursor] = !fs.selected[fs.cursor];
                        }
                    }
                    input::CANCEL => {
                        self.state = State::Browsing(Self::make_browse_state(&self.ros));
                    }
                    // Any other key treated as "start plotting" shortcut
                    // Use ZOOM_IN as "start" since it's a distinct unused key here
                    input::ZOOM_IN => {
                        // pull out the FieldState to move it
                        if let State::SelectingFields(_) = &self.state {
                            let fs = match std::mem::replace(
                                &mut self.state,
                                State::Browsing(Self::make_browse_state(&self.ros)),
                            ) {
                                State::SelectingFields(fs) => fs,
                                _ => unreachable!(),
                            };
                            self.start_plotting(fs);
                        }
                    }
                    _ => {}
                }
            }

            // ── Plotting ──────────────────────────────────────────────────────
            State::Plotting(ps) => {
                match action {
                    input::LEFT => {
                        if !ps.visible.is_empty() {
                            ps.field_cursor = if ps.field_cursor > 0 {
                                ps.field_cursor - 1
                            } else {
                                ps.visible.len() - 1
                            };
                        }
                    }
                    input::RIGHT => {
                        if !ps.visible.is_empty() {
                            ps.field_cursor = (ps.field_cursor + 1) % ps.visible.len();
                        }
                    }
                    input::CONFIRM => {
                        if ps.field_cursor < ps.visible.len() {
                            ps.visible[ps.field_cursor] = !ps.visible[ps.field_cursor];
                        }
                    }
                    input::ZOOM_IN  => { ps.window_secs += 5.0; }
                    input::ZOOM_OUT => { ps.window_secs = (ps.window_secs - 5.0).max(5.0); }
                    input::CANCEL => {
                        self.state = State::Browsing(Self::make_browse_state(&self.ros));
                    }
                    _ => {}
                }
            }
        }
    }

    fn get_name(&self) -> String { "Plot".to_string() }

    fn get_description(&self) -> Vec<String> {
        vec![
            "Browse available ROS topics and plot numeric fields in real time.".to_string(),
            "Browsing: up/down to navigate, Enter to select topic, Esc to refresh list.".to_string(),
            "Fields: up/down to navigate, Enter to toggle, + to start plotting, Esc to go back.".to_string(),
            "Plotting: left/right to move cursor, Enter to toggle field, +/- for window, Esc to go back.".to_string(),
        ]
    }

    fn get_keymap(&self) -> Vec<[String; 2]> {
        vec![
            [input::UP.to_string(),       "Navigate up".to_string()],
            [input::DOWN.to_string(),     "Navigate down".to_string()],
            [input::LEFT.to_string(),     "Move field cursor left (plotting)".to_string()],
            [input::RIGHT.to_string(),    "Move field cursor right (plotting)".to_string()],
            [input::CONFIRM.to_string(),  "Select topic / toggle field".to_string()],
            [input::ZOOM_IN.to_string(),  "Start plotting (field selection) / widen window".to_string()],
            [input::ZOOM_OUT.to_string(), "Narrow time window (plotting)".to_string()],
            [input::CANCEL.to_string(),   "Go back / refresh topic list".to_string()],
        ]
    }
}

// ── Drawable ──────────────────────────────────────────────────────────────────

impl Drawable for PlotView {
    fn draw(&self, f: &mut Frame) {
        let area = f.area();
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Length(3), Constraint::Min(0)])
            .split(area);

        match &self.state {
            State::Browsing(bs) => draw_browsing(f, chunks[0], chunks[1], bs),
            State::SelectingFields(fs) => draw_fields(f, chunks[0], chunks[1], fs),
            State::Plotting(ps) => draw_plotting(f, chunks[0], chunks[1], ps),
        }
    }
}

// ── drawing helpers ───────────────────────────────────────────────────────────

fn title_bar(f: &mut Frame, area: ratatui::layout::Rect, subtitle: &str) {
    let line = TextLine::from(vec![
        Span::styled("Plot", Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)),
        Span::raw(format!("  {}", subtitle)),
    ]);
    f.render_widget(
        Paragraph::new(line)
            .block(Block::default().borders(Borders::ALL))
            .alignment(Alignment::Center),
        area,
    );
}

fn draw_browsing(
    f: &mut Frame,
    title_area: ratatui::layout::Rect,
    list_area: ratatui::layout::Rect,
    bs: &BrowsingState,
) {
    title_bar(f, title_area, "select topic  (↑↓ navigate  Enter select  Esc refresh)");

    if bs.topics.is_empty() {
        f.render_widget(
            Paragraph::new("No supported topics found. Press Esc to refresh.")
                .style(Style::default().fg(Color::DarkGray))
                .alignment(Alignment::Center)
                .block(Block::default().borders(Borders::ALL)),
            list_area,
        );
        return;
    }

    let items: Vec<ListItem> = bs.topics.iter().enumerate().map(|(i, (name, _type, short))| {
        let style = if i == bs.cursor {
            Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD | Modifier::REVERSED)
        } else {
            Style::default().fg(Color::White)
        };
        ListItem::new(TextLine::from(vec![
            Span::styled(format!("{:<40}", name), style),
            Span::styled(format!(" [{}]", short), Style::default().fg(Color::DarkGray)),
        ]))
    }).collect();

    let mut state = ListState::default();
    state.select(Some(bs.cursor));

    f.render_stateful_widget(
        List::new(items).block(Block::default().borders(Borders::ALL).title(" Available topics ")),
        list_area,
        &mut state,
    );
}

fn draw_fields(
    f: &mut Frame,
    title_area: ratatui::layout::Rect,
    list_area: ratatui::layout::Rect,
    fs: &FieldState,
) {
    title_bar(f, title_area, &format!(
        "{}  [{}]  (↑↓ navigate  Enter toggle  + start  Esc back)",
        fs.topic, fs.msg_type
    ));

    let items: Vec<ListItem> = fs.fields.iter().enumerate().map(|(i, &field)| {
        let check = if fs.selected[i] { "▣" } else { "□" };
        let color = PALETTE[i % PALETTE.len()];
        let style = if i == fs.cursor {
            Style::default().fg(color).add_modifier(Modifier::BOLD | Modifier::REVERSED)
        } else {
            Style::default().fg(color)
        };
        ListItem::new(TextLine::from(vec![
            Span::styled(format!(" {} {}", check, field), style),
        ]))
    }).collect();

    let mut state = ListState::default();
    state.select(Some(fs.cursor));

    f.render_stateful_widget(
        List::new(items).block(Block::default().borders(Borders::ALL).title(" Select fields to plot ")),
        list_area,
        &mut state,
    );
}

fn draw_plotting(
    f: &mut Frame,
    title_area: ratatui::layout::Rect,
    content_area: ratatui::layout::Rect,
    ps: &PlottingState,
) {
    title_bar(f, title_area, &format!(
        "{}  window {:.0} s  (← → cursor  Enter toggle  +/- window  Esc back)",
        ps.listener.topic, ps.window_secs
    ));

    // Split content into chart + legend bar
    let sub = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(0), Constraint::Length(3)])
        .split(content_area);

    // Legend bar
    let mut legend_spans = vec![Span::raw(" ")];
    for (fi, field) in ps.listener.fields.iter().enumerate() {
        let color = PALETTE[fi % PALETTE.len()];
        let check = if ps.visible[fi] { "▣" } else { "□" };
        let label = format!("{} {}", check, field);
        let style = if fi == ps.field_cursor {
            Style::default().fg(color).add_modifier(Modifier::BOLD | Modifier::REVERSED)
        } else {
            Style::default().fg(color)
        };
        legend_spans.push(Span::styled(label, style));
        legend_spans.push(Span::raw("  "));
    }
    f.render_widget(
        Paragraph::new(TextLine::from(legend_spans))
            .block(Block::default().borders(Borders::ALL)),
        sub[1],
    );

    // Chart
    let now = ps.start.elapsed().as_secs_f64();
    let x_min = (now - ps.window_secs).max(0.0);
    let x_max = now;

    let data_vecs: Vec<Vec<(f64, f64)>> = (0..ps.listener.fields.len())
        .map(|fi| {
            if fi >= ps.buffers.len() { return vec![]; }
            ps.buffers[fi].iter().filter(|(t, _)| *t >= x_min).copied().collect()
        })
        .collect();

    let (raw_min, raw_max) = data_vecs.iter().enumerate()
        .filter(|(fi, _)| ps.visible.get(*fi).copied().unwrap_or(false))
        .flat_map(|(_, pts)| pts.iter().map(|(_, v)| *v))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(mn, mx), v| (mn.min(v), mx.max(v)));

    let (y_min, y_max) = if raw_min.is_infinite() || raw_max.is_infinite() {
        (-1.0, 1.0)
    } else {
        let centre = (raw_min + raw_max) / 2.0;
        let half = ((raw_max - raw_min) / 2.0).max(0.1);
        let pad = half * 0.1;
        (centre - half - pad, centre + half + pad)
    };

    let datasets: Vec<Dataset> = (0..ps.listener.fields.len())
        .filter(|&fi| ps.visible.get(fi).copied().unwrap_or(false))
        .map(|fi| {
            Dataset::default()
                .name(ps.listener.fields[fi].as_str())
                .marker(symbols::Marker::Braille)
                .style(Style::default().fg(PALETTE[fi % PALETTE.len()]))
                .data(&data_vecs[fi])
        })
        .collect();

    let chart = Chart::new(datasets)
        .block(Block::default().borders(Borders::ALL))
        .x_axis(
            Axis::default()
                .title("t (s)")
                .style(Style::default().fg(Color::DarkGray))
                .bounds([x_min, x_max])
                .labels(vec![
                    Span::raw(format!("{:.0}", x_min)),
                    Span::raw(format!("{:.0}", (x_min + x_max) / 2.0)),
                    Span::raw(format!("{:.0}", x_max)),
                ]),
        )
        .y_axis(
            Axis::default()
                .style(Style::default().fg(Color::DarkGray))
                .bounds([y_min, y_max])
                .labels(vec![
                    Span::raw(format!("{:.2}", y_min)),
                    Span::raw(format!("{:.2}", (y_min + y_max) / 2.0)),
                    Span::raw(format!("{:.2}", y_max)),
                ]),
        );

    f.render_widget(chart, sub[0]);
}

impl BaseMode for PlotView {}
