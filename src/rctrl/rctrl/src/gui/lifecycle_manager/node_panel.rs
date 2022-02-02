use crate::gui::lifecycle_manager::state_display::StateDisplay;
use crate::gui::GuiElem;
use crate::ws_lock::WsLock;
use eframe::egui;
use serde_json::Value;
use std::rc::Rc;
use std::sync::Mutex;

pub struct NodePanel {
    ws: Rc<WsLock>,
    pub node: String,
    pub state_display: Rc<Mutex<StateDisplay>>,
}

impl Default for NodePanel {
    fn default() -> Self {
        Self {
            ws: Rc::default(),
            node: String::new(),
            state_display: Rc::default(),
        }
    }
}

impl NodePanel {
    pub fn new(node: String, ws: &Rc<WsLock>) -> Self {
        let mut node_panel = Self::default();
        node_panel.ws = Rc::clone(&ws);
        node_panel.node = node;
        node_panel.state_display = StateDisplay::new_shared(node_panel.node.as_ref(), ws);

        return node_panel;
    }
}

impl GuiElem for NodePanel {
    fn draw(&self, ui: &mut egui::Ui) {
        ui.label(format!("{:?}", self.node));
        self.state_display.lock().unwrap().draw(ui);
    }

    fn update_data(&mut self, data: Value) {}
}
