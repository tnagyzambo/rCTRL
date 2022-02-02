// use eframe::egui;
// use egui::Ui;
// use rctrl_rosbridge::lifecycle_msgs::msg::State;
// use serde_json::Value;
// use std::collections::HashMap;
// use std::collections::VecDeque;
// use std::rc::Rc;
// use tokio::sync::RwLock;
// use wasm_bindgen_futures::spawn_local;

// use crate::{GuiElem, GuiElems};

// struct LifecyclePanels {
//     panel_map: HashMap<String, StatePanel>,
// }

// impl Default for LifecyclePanels {
//     fn default() -> Self {
//         Self {
//             panel_map: HashMap::new(),
//         }
//     }
// }

// impl GuiElem for LifecyclePanels {
//     fn draw(
//         &self,
//         ui: &mut Ui,
//         ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>,
//         ws_write_lock: &Rc<RwLock<VecDeque<String>>>,
//     ) {
//         let tooltip = "Refresh ROS2 node list";
//         if ui.button("🔄").on_hover_text(tooltip).clicked() {
//             let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new("/rosapi/nodes");
//             let mut w = ws_write_lock.write().unwrap();
//             w.push_back(serde_json::to_string(&cmd).unwrap());
//         };

//         for (key, value) in &self.panel_map {
//             value.connect_to_ws(ws_read_lock);
//             value.draw(ui, ws_read_lock, ws_write_lock);
//         }
//     }

//     fn update_data(&mut self, data: Value) {
//         let deserialized: rctrl_rosbridge::rosapi_msgs::srv::nodes::Response =
//             serde_json::from_value(data).unwrap();

//         self.panel_map.retain(|k, _| deserialized.nodes.contains(k));

//         for node in deserialized.nodes {
//             self.panel_map
//                 .entry(node.clone())
//                 .or_insert_with(|| StatePanel::new(node, ws_read_lock));
//         }
//     }
// }

// pub struct StatePanel {
//     node: String,
//     state: StateDisplay,
// }

// impl StatePanel {
//     fn new(node: &'static str, ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>) -> Self {
//         Self {
//             node: node.to_string(),
//             state: StateDisplay::new(node, ws_read_lock),
//         }
//     }
// }

// impl GuiElem for StatePanel {
//     fn draw(
//         &self,
//         ui: &mut Ui,
//         ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>,
//         ws_write_lock: &Rc<RwLock<VecDeque<String>>>,
//     ) {
//         ui.label(&self.node);
//         self.state.draw(ui, ws_read_lock, ws_write_lock);
//     }

//     fn update_data(&mut self, data: Value) {}
// }
