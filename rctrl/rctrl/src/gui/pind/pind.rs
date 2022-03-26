use crate::gui::{gui_elem::gen_gui_elem_id, gui_elem::GuiElem, App, RStatePanel};
use crate::ws_lock::WsLock;
use eframe::{egui, epi};
use gloo_console::log;
use std::rc::Rc;

pub struct PInD {
    ws_lock: Rc<WsLock>,
    rstate_panel: RStatePanel,
}

impl PInD {
    pub fn new(ws_lock: &Rc<WsLock>) -> Self {
        Self {
            ws_lock: Rc::clone(&ws_lock),
            rstate_panel: RStatePanel::new(ws_lock),
        }
    }
}

impl epi::App for PInD {
    fn name(&self) -> &str {
        "pind"
    }

    /// Called once before the first frame.
    fn setup(&mut self, _ctx: &egui::Context, _frame: &epi::Frame, _storage: Option<&dyn epi::Storage>) {
        // Load previous app state (if any).
        // Note that you must enable the `persistence` feature for this to work.
        #[cfg(feature = "persistence")]
        if let Some(storage) = _storage {
            *self = epi::get_value(storage, epi::APP_KEY).unwrap_or_default()
        }
    }

    /// Called by the frame work to save state before shutdown.
    /// Note that you must enable the `persistence` feature for this to work.
    #[cfg(feature = "persistence")]
    fn save(&mut self, storage: &mut dyn epi::Storage) {
        epi::set_value(storage, epi::APP_KEY, self);
    }

    /// Called each time the UI needs repainting, which may be many times per second.
    /// Put your widgets into a `SidePanel`, `TopPanel`, `CentralPanel`, `Window` or `Area`.
    fn update(&mut self, ctx: &egui::Context, frame: &epi::Frame) {
        let Self { ws_lock, rstate_panel } = self;

        egui::CentralPanel::default().show(ctx, |ui| {
            let rstate_panel = self.rstate_panel.draw(ctx, ui);
        });
    }
}

impl App for PInD {
    fn refresh(&mut self) {
        // Call for refresh
        // Request state of node
        let topic = "/rstate/get_state";
        let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
        self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

        // Request available transitions of node
        let topic = "/rstate/get_available_transitions";
        let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
        self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

        let topic = "/rstate/get_network_state";
        let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
        self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

        // Request available transitions of node
        let topic = "/rstate/get_available_network_transitions";
        let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
        self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
    }
}
