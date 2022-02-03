use crate::gui::gui_elem::GuiElem;
use crate::ws_lock::WsLock;
use eframe::egui;
use gloo_console::log;
use rctrl_rosbridge::lifecycle_msgs::msg::State;
use serde_json::Value;
use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Mutex;
use web_sys::{Window, Performance};

/// Main [`LifecycleManager`] structure.
pub struct LifecycleManager {
    ws: Rc<WsLock>,
    pub open: bool,
    window: Window,
    performance: Performance,
    last_refresh: Option<f64>,
    node_panels: Rc<Mutex<HashMap<String, NodePanel>>>,
}

impl LifecycleManager {
    pub fn new_shared(ws: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let window = web_sys::window().expect("should have a window in this context");
        let performance = window
            .performance()
            .expect("performance should be available");

        let lifcycle_manager = Self {
            ws: Rc::clone(&ws),
            open: true,
            window: window,
            performance: performance,
            last_refresh: None,
            node_panels: Rc::new(Mutex::new(HashMap::new())),
        };

        let op = ("service_response").to_owned();
        let topic = ("/rosapi/nodes").to_owned();
        let lifcycle_manager_lock = Rc::new(Mutex::new(lifcycle_manager));
        let lifcycle_manager_lock_c = Rc::clone(&lifcycle_manager_lock);
        ws.add_gui_elem(op, topic, lifcycle_manager_lock_c);

        return lifcycle_manager_lock;
    }
}

impl GuiElem for LifecycleManager {
    fn draw(&self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let tooltip = "Refresh ROS2 node list";
            if ui.button("🔄").on_hover_text(tooltip).clicked() {
                let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new("/rosapi/nodes");
                self.ws.add_ws_write(serde_json::to_string(&cmd).unwrap());
            }

            let refresh_msg = match self.last_refresh {
                Some(last_refresh) => format!("Refreshed {}ms ago", self.performance.now() - last_refresh),
                None => "Refreshed N/A ago".to_string(),
            };

            ui.with_layout(egui::Layout::right_to_left(), |ui| {
                ui.label(refresh_msg);
            });
        });

        ui.separator();

        ui.horizontal_wrapped(|ui| {
            for (key, value) in self.node_panels.lock().unwrap().iter() {
                ui.label(format!(""));
                value.draw(ui);

                // Break wrap if available space is smaller than the width of the next panel
                if ui.available_size_before_wrap().x <= 150.0 {
                    ui.end_row();
                }        
            }
        });
    }

    fn update_data(&mut self, data: Value) {
        // Get comlete list of nodes
        let deserialized: rctrl_rosbridge::rosapi_msgs::srv::nodes::Response = serde_json::from_value(data).unwrap();

        // Retain all node_panels whos names exist in the respone, drop all others
        self.node_panels
            .lock()
            .unwrap()
            .retain(|k, _| deserialized.nodes.contains(k));

        // Create and insert new node_panels if they do node exist in the map
        for node in deserialized.nodes {
            self.node_panels
                .lock()
                .unwrap()
                .entry(node.clone())
                .or_insert_with(|| NodePanel::new(node.clone(), &self.ws));

            // Request state of node
            let topic = node + "/get_state";
            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
            self.ws.add_ws_write(serde_json::to_string(&cmd).unwrap());
        }

        // Update last refresh counter
        self.last_refresh = Some(self.performance.now());
    }
}

/// Wrapper structure containing all [`GuiElem`]'s needed for a single node.
pub struct NodePanel {
    ws: Rc<WsLock>,
    pub node: String,
    pub state_display: Rc<Mutex<StateDisplay>>,
}

impl NodePanel {
    pub fn new(node: String, ws: &Rc<WsLock>) -> Self {
        let state_display = StateDisplay::new_shared(node.clone(), ws);

        Self {
            ws: Rc::clone(&ws),
            node: node,
            state_display: state_display,
        }
    }
}

impl GuiElem for NodePanel {
    fn draw(&self, ui: &mut egui::Ui) {
        ui.group(|ui| {
            ui.set_width(150.0);
            ui.vertical_centered(|ui| {
                ui.label(format!("{:?}", self.node));
                self.state_display.lock().unwrap().draw(ui);
            });
            
        });
    }

    fn update_data(&mut self, data: Value) {}
}

/// Display the [`State`] of a node.
pub struct StateDisplay {
    ws: Rc<WsLock>,
    state: State,
}

impl StateDisplay {
    pub fn new_shared(node: String, ws: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let state_display = Self {
            ws: Rc::clone(&ws),
            state: State::Unknown,
        };

        let op = ("service_response").to_owned();
        let topic = node + "/get_state";
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
        match serde_json::from_value::<rctrl_rosbridge::lifecycle_msgs::srv::get_state::Response>(data) {
            Ok(values) => self.state = values.current_state,
            Err(e) => {
                self.state = State::Unknown;
                log!(format!("StateDisplay::update_data() failed: {}", e));
            }
        }
    }
}
