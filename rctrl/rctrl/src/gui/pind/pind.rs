use crate::gui::{App, FireButton, RStatePanel, ValveControl};
use crate::ws_lock::WsLock;
use eframe::{egui, epi};
use rctrl_rosbridge::rstate_msgs::msg::NetworkState;
use std::rc::Rc;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
extern "C" {
    type Buffer;
}

#[wasm_bindgen(module = "fs")]
extern "C" {
    #[wasm_bindgen(js_name = readFileSync, catch)]
    fn read_file(path: &str) -> Result<Buffer, JsValue>;
}

pub struct PInD {
    ws_lock: Rc<WsLock>,
    rstate_panel: RStatePanel,
    texture: Option<egui::TextureHandle>,
    valve_control_mv1: ValveControl,
    valve_control_mv2: ValveControl,
    valve_control_esv: ValveControl,
    valve_control_pv: ValveControl,
    fire_button: FireButton,
}

impl PInD {
    pub fn new(ws_lock: &Rc<WsLock>) -> Self {
        Self {
            ws_lock: Rc::clone(&ws_lock),
            rstate_panel: RStatePanel::new(ws_lock),
            texture: None,
            valve_control_mv1: ValveControl::new(ws_lock, "mv1".to_string(), "MV1".to_string()),
            valve_control_mv2: ValveControl::new(ws_lock, "mv2".to_string(), "MV2".to_string()),
            valve_control_esv: ValveControl::new(ws_lock, "bv".to_string(), "BV".to_string()),
            valve_control_pv: ValveControl::new(ws_lock, "pv".to_string(), "PV".to_string()),
            fire_button: FireButton::new(ws_lock),
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
        let Self {
            ws_lock,
            rstate_panel,
            texture,
            valve_control_mv1,
            valve_control_mv2,
            valve_control_esv,
            valve_control_pv,
            fire_button,
        } = self;

        egui::CentralPanel::default().show(ctx, |ui| {
            let texture: &egui::TextureHandle = self.texture.get_or_insert_with(|| {
                // Can not easily access filesystem from wasm, load at compile time
                let bytes = include_bytes!("pnid.png");
                let image = load_image_from_memory(bytes);
                // Load the texture only once.
                ui.ctx().load_texture("pnid", image.unwrap())
            });

            ui.add_space(100.0);
            ui.horizontal(|ui| {
                ui.add_space(250.0);
                ui.image(texture, egui::Vec2::new(500.0, 500.0));
            });

            egui::Area::new("pnid_overlay").fixed_pos(egui::pos2(32.0, 32.0)).show(ctx, |ui| {
                ui.horizontal(|ui| {
                    let rstate_panel = self.rstate_panel.draw(ctx, ui);
                    ui.add_space(580.0);
                    self.fire_button.draw(ctx, ui);
                });

                ui.add_space(50.0);
                ui.horizontal(|ui| {
                    ui.add_space(200.0);
                    self.valve_control_esv.draw(ctx, ui);
                    ui.add_space(200.0);
                    self.valve_control_pv.draw(ctx, ui);
                });
                ui.add_space(220.0);
                ui.horizontal(|ui| {
                    ui.add_space(240.0);
                    self.valve_control_mv1.draw(ctx, ui);
                    ui.add_space(165.0);
                    self.valve_control_mv2.draw(ctx, ui);
                });
            });
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

        let network_state = &self.rstate_panel.rstate_network_state.lock().unwrap().state;
        if *network_state == NetworkState::Active {
            // Request valve state
            let topic = "/recu/mv1/get_state";
            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
            self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

            // Request valve state
            let topic = "/recu/mv2/get_state";
            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
            self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

            // Request valve state
            let topic = "/recu/bv/get_state";
            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
            self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

            // Request valve state
            let topic = "/recu/pv/get_state";
            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
            self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
        }
    }
}

fn load_image_from_memory(image_data: &[u8]) -> Result<egui::ColorImage, image::ImageError> {
    let image = image::load_from_memory(image_data)?;
    let size = [image.width() as _, image.height() as _];
    let image_buffer = image.to_rgba8();
    let pixels = image_buffer.as_flat_samples();
    Ok(egui::ColorImage::from_rgba_unmultiplied(size, pixels.as_slice()))
}
