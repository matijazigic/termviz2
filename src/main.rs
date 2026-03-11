mod app;
mod config;
mod inputs;
mod modes;
mod outputs;
mod ros;
mod terminal;
mod utils;

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use clap::{value_parser, Arg, Command};
use crossterm::event::{Event, EventStream, KeyCode, KeyEvent, KeyModifiers};
use futures::{future::FutureExt, select, StreamExt};
use futures_timer::Delay;
use std::error::Error;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let matches = Command::new("termviz2")
        .about("ROS2 visualization on the terminal")
        .arg(
            Arg::new("config")
                .long("config")
                .short('c')
                .long_help("Optional YAML file with a custom termviz2 configuration.")
                .value_parser(value_parser!(PathBuf)),
        )
        .after_help("More documentation can be found at: https://github.com/matijazigic/termviz2")
        .get_matches();

    let conf: config::Termviz2Config = config::get_config(matches.get_one("config"))?;

    let rate: Duration = Duration::from_millis(1000 / conf.target_framerate);

    println!("Initializing ROS...");
    let ros = Arc::new(ros::ROS::new(&conf.tf_topic, &conf.tf_static_topic)?);

    println!("Waiting for 2 TF messages...");
    ros.wait_for_n_messages(2);

    println!("Waiting for transform {} -> {}...", conf.fixed_frame, conf.robot_frame);
    let timeout = Duration::from_secs(5);
    if !ros.wait_for_transform(&conf.fixed_frame, &conf.robot_frame, timeout) {
        eprintln!("Error: transform {} -> {} not available after {}s. Is TF being published?",
            conf.fixed_frame, conf.robot_frame, timeout.as_secs());
        return Err("Robot pose not available on TF.".into());
    }

    println!("Initializing app...");
    let mut app = app::App::new(Arc::clone(&ros), conf);
    println!("Initializing terminal...");
    let mut terminal = terminal::init()?;
    let mut reader = EventStream::new();

    let result = run_app(&mut terminal, &mut reader, &mut app, rate).await;
    
    terminal::restore(&mut terminal)?;
    println!("Restored terminal and dropping ROS...");
    drop(app);
    drop(ros);

    if let Err(e) = result {
        eprintln!("Error: {:?}", e);
    }

    Ok(())
}

async fn run_app(
    terminal: &mut terminal::Term,
    reader: &mut EventStream,
    app: &mut app::App,
    rate: Duration,
) -> Result<(), Box<dyn Error>> {
    loop {
        let mut event = reader.next().fuse();
        let mut delay = Delay::new(rate).fuse();

        select! {
            _ = delay => {
                app.run();
            },
            maybe_event = event => {
                match maybe_event {
                    Some(Ok(event)) => {
                        // Ctrl+C always quits
                        if event == Event::Key(KeyEvent {
                            code: KeyCode::Char('c'),
                            modifiers: KeyModifiers::CONTROL,
                            kind: crossterm::event::KeyEventKind::Press,
                            state: crossterm::event::KeyEventState::NONE,
                        }) {
                            break;
                        }

                        if let Event::Key(input) = event {
                            // Digit keys switch modes directly (1-based index).
                            if let KeyCode::Char(c) = input.code {
                                if c.is_ascii_digit() {
                                    app.handle_input(&c.to_string());
                                    continue;
                                }
                            }
                            if let Some(action) = app.key_to_action.get(&input.code).cloned() {
                                app.handle_input(&action);
                            }
                        }
                    }
                    Some(Err(e)) => eprintln!("Input error: {:?}\r", e),
                    None => break,
                }
            }
        };

        terminal.draw(|f| {
            app.draw(f);
        })?;
    }

    Ok(())
}

