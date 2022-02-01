use eframe::egui;
use egui::Ui;
use rctrl_rosbridge::lifecycle_msgs::msg::State;
use serde_json::Value;
use std::collections::HashMap;
use std::collections::VecDeque;
use std::rc::Rc;
use std::sync::RwLock;

use crate::{GuiElem, GuiElems};

pub struct LifecycleManager {
    lifecycle_panels: Rc<RwLock<dyn GuiElem>>,
}

impl Default for LifecycleManager {
    fn default() -> Self {
        Self {
            lifecycle_panels: Rc::new(RwLock::new(LifecyclePanels::default())),
        }
    }
}

impl LifecycleManager {
    pub fn connect_to_ws(&self, ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>) {
        let mut hash_map = ws_read_lock.write().unwrap();
        hash_map.insert(
            String::from("service_response"),
            String::from("/rosapi/nodes"),
            Rc::clone(&self.lifecycle_panels),
        );
    }

    pub fn draw(
        &mut self,
        ctx: &egui::CtxRef,
        ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>,
        ws_write_lock: &Rc<RwLock<VecDeque<String>>>,
    ) {
        egui::Window::new("Lifecycle Manager")
            .open(&mut true)
            .resizable(true)
            .default_width(520.0)
            .show(ctx, |ui| {
                self.lifecycle_panels
                    .write()
                    .unwrap()
                    .draw(ui, ws_read_lock, ws_write_lock);
            });
    }
}

struct LifecyclePanels {
    panel_map: HashMap<String, StatePanel>,
}

impl Default for LifecyclePanels {
    fn default() -> Self {
        Self {
            panel_map: HashMap::new(),
        }
    }
}

impl GuiElem for LifecyclePanels {
    fn draw(
        &self,
        ui: &mut Ui,
        ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>,
        ws_write_lock: &Rc<RwLock<VecDeque<String>>>,
    ) {
        let tooltip = "Refresh ROS2 node list";
        if ui.button("🔄").on_hover_text(tooltip).clicked() {
            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new("/rosapi/nodes");
            let mut w = ws_write_lock.write().unwrap();
            w.push_back(serde_json::to_string(&cmd).unwrap());
        };

        for (key, value) in &self.panel_map {
            value.connect_to_ws(ws_read_lock);
            value.draw(ui, ws_read_lock, ws_write_lock);
        }
    }

    fn update_data(&mut self, data: Value) {
        let deserialized: rctrl_rosbridge::rosapi_msgs::srv::nodes::Response =
            serde_json::from_value(data).unwrap();

        self.panel_map.retain(|k, _| deserialized.nodes.contains(k));

        for node in deserialized.nodes {
            self.panel_map
                .entry(node.clone())
                .or_insert_with(|| StatePanel::new(node));
        }
    }
}

pub struct StatePanel {
    node: String,
    state: Rc<RwLock<dyn GuiElem>>,
}

impl StatePanel {
    fn new(node: String) -> Self {
        Self {
            node: node,
            state: Rc::new(RwLock::new(StateDisplay::default())),
        }
    }

    pub fn connect_to_ws(&self, ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>) {
        let mut hash_map = ws_read_lock.write().unwrap();
        hash_map.insert(
            String::from("service_response"),
            String::from(String::new() + &self.node + "/get_state"),
            Rc::clone(&self.state),
        );
    }
}

impl GuiElem for StatePanel {
    fn draw(
        &self,
        ui: &mut Ui,
        ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>,
        ws_write_lock: &Rc<RwLock<VecDeque<String>>>,
    ) {
        ui.label(&self.node);
        self.state
            .read()
            .unwrap()
            .draw(ui, ws_read_lock, ws_write_lock);
    }

    fn update_data(&mut self, data: Value) {}
}

pub struct StateDisplay {
    state: State,
}

impl StateDisplay {
    pub fn default() -> Self {
        Self {
            state: State::Unknown,
        }
    }
}

impl GuiElem for StateDisplay {
    fn draw(
        &self,
        ui: &mut Ui,
        ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>,
        ws_write_lock: &Rc<RwLock<VecDeque<String>>>,
    ) {
        ui.label(format!("{:?}", self.state));
    }

    fn update_data(&mut self, data: Value) {
        let values: rctrl_rosbridge::lifecycle_msgs::srv::get_state::Response =
            serde_json::from_value(data).unwrap();
        self.state = values.current_state;
    }
}
