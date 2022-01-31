/// REFERENCE: <https://stackoverflow.com/questions/45786717/how-to-implement-hashmap-with-two-keys/45795699>
use rctrl_rosbridge::lifecycle_msgs::msg::State;
use serde_json::Value;
use eframe::{egui, epi};

use crate::GuiElem;

pub struct StateDisplay {
    state: State,
}

impl StateDisplay {
    pub fn new() -> Self {
        Self { 
            state: State::Unknown 
        }
    }
}

impl GuiElem for StateDisplay {
    fn update_gui(&self, ctx: &egui::CtxRef, frame: &epi::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            // The top panel is often a good place for a menu bar:
            egui::menu::bar(ui, |ui| {
                ui.label(format!("{:?}", self.state));
            });
        });
    }

    fn update_data(&mut self, data: Value) {
        let values: rctrl_rosbridge::lifecycle_msgs::srv::get_state::Response = serde_json::from_value(data).unwrap();
        self.state = values.current_state;
    }
}
