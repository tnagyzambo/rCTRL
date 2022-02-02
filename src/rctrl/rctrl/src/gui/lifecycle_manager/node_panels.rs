use crate::gui::lifecycle_manager::node_panel::NodePanel;
use crate::gui::GuiElem;
use crate::ws_lock::WsLock;
use eframe::egui;
use serde_json::Value;
use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Mutex;

pub struct NodePanels {
    ws: Rc<WsLock>,
    node_panels: HashMap<String, NodePanel>,
}

impl NodePanels {
    pub fn new_shared(ws: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let node_panels = Self {
            ws: Rc::clone(&ws),
            node_panels: HashMap::new(),
        };

        let op = ("service_response").to_owned();
        let topic = ("/rosapi/nodes").to_owned();
        let node_panels_lock = Rc::new(Mutex::new(node_panels));
        let node_panels_lock_c = Rc::clone(&node_panels_lock);
        ws.add_gui_elem(op, topic, node_panels_lock_c);

        return node_panels_lock;
    }
}

impl GuiElem for NodePanels {
    fn draw(&self, ui: &mut egui::Ui) {
        let tooltip = "Refresh ROS2 node list";
        if ui.button("🔄").on_hover_text(tooltip).clicked() {
            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new("/rosapi/nodes");
            self.ws.add_ws_write(serde_json::to_string(&cmd).unwrap());
        }

        for (key, value) in &self.node_panels {
            value.draw(ui);
        }
    }

    fn update_data(&mut self, data: Value) {
        let deserialized: rctrl_rosbridge::rosapi_msgs::srv::nodes::Response =
            serde_json::from_value(data).unwrap();

        self.node_panels
            .retain(|k, _| deserialized.nodes.contains(k));

        for node in deserialized.nodes {
            let ws = &self.ws;
            self.node_panels
                .entry(node.clone())
                .or_insert_with(|| NodePanel::new(node, ws));
        }
    }
}
