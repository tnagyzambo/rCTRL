use crate::gui::GuiElem;
use crate::ws_lock::WsLock;
use eframe::egui;
use rctrl_rosbridge::lifecycle_msgs::msg::State;
use serde_json::Value;
use std::rc::Rc;
use std::sync::Mutex;

pub struct StateDisplay {
    ws: Rc<WsLock>,
    state: State,
}

impl Default for StateDisplay {
    fn default() -> Self {
        Self {
            ws: Rc::default(),
            state: State::Unknown,
        }
    }
}

impl StateDisplay {
    pub fn new_shared(node: &str, ws: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let mut state_display = Self::default();
        state_display.ws = Rc::clone(&ws);
        let op = ("service_response").to_owned();
        let topic = node.to_owned() + "/get_state";
        let state_display_lock = Rc::new(Mutex::new(state_display));
        let state_display_lock_c = Rc::clone(&state_display_lock);
        ws.add_gui_elem(op, topic, state_display_lock_c);

        return state_display_lock;
    }
}

impl GuiElem for StateDisplay {
    fn draw(&self, ui: &mut egui::Ui) {
        ui.label(format!("{:?}", self.state));
    }

    fn update_data(&mut self, data: Value) {
        let values: rctrl_rosbridge::lifecycle_msgs::srv::get_state::Response =
            serde_json::from_value(data).unwrap();
        self.state = values.current_state;
    }
}
