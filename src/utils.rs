use std::collections::HashMap;

use crossterm::event::KeyCode;

/// Converts the config key mapping (action -> key string) into
/// a runtime lookup map (KeyCode -> action).
pub fn build_key_map(key_mapping: &HashMap<String, String>) -> HashMap<KeyCode, String> {
    key_mapping
        .iter()
        .map(|(action, key)| match key.as_str() {
            "Enter" => (KeyCode::Enter, action.clone()),
            "Esc" => (KeyCode::Esc, action.clone()),
            "Space" => (KeyCode::Char(' '), action.clone()),
            s => (KeyCode::Char(s.chars().next().unwrap()), action.clone()),
        })
        .collect()
}
