use crate::gui::lifecycle_manager::node_panels::NodePanels;
use crate::gui::GuiElem;
use crate::ws_lock::WsLock;
use eframe::egui;
use std::rc::Rc;
use std::sync::Mutex;

pub struct LifecycleManager {
    ws: Rc<WsLock>,
    node_panels: Rc<Mutex<NodePanels>>,
}

impl LifecycleManager {
    pub fn new(ws: &Rc<WsLock>) -> Self {
        Self {
            ws: Rc::clone(&ws),
            node_panels: NodePanels::new_shared(ws),
        }
    }

    pub fn draw(&self, ctx: &egui::CtxRef) {
        egui::Window::new("Lifecycle Manager")
            .open(&mut true)
            .resizable(true)
            .default_width(520.0)
            .show(&ctx, |ui| {
                self.node_panels.lock().unwrap().draw(ui);
            });
    }
}
