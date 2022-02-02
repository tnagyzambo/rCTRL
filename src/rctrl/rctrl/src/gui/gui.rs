use crate::gui::lifecycle_manager::LifecycleManager;
use crate::ws_lock::WsLock;
use eframe::{egui, epi};
use std::rc::Rc;

/// We derive Deserialize/Serialize so we can persist app state on shutdown.
#[cfg_attr(feature = "persistence", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "persistence", serde(default))] // if we add new fields, give them default values when deserializing old state
pub struct Gui {
    ws: Rc<WsLock>,
    lifecycle_manager: LifecycleManager,
}

impl Gui {
    pub fn new(ws: Rc<WsLock>) -> Self {
        let lifecycle_manager = LifecycleManager::new(&ws);
        Self {
            ws: ws,
            lifecycle_manager: lifecycle_manager,
        }
    }
}

impl epi::App for Gui {
    fn name(&self) -> &str {
        "rctrl"
    }

    /// Called once before the first frame.
    fn setup(
        &mut self,
        _ctx: &egui::CtxRef,
        _frame: &epi::Frame,
        _storage: Option<&dyn epi::Storage>,
    ) {
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
    fn update(&mut self, ctx: &egui::CtxRef, frame: &epi::Frame) {
        let Self {
            ws,
            lifecycle_manager,
        } = self;
        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            // The top panel is often a good place for a menu bar:
            egui::menu::bar(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("Quit").clicked() {
                        frame.quit();
                    }
                });
            });
        });

        egui::SidePanel::left("side_panel").show(ctx, |ui| {
            ui.heading("Side Panel");

            if ui.button("Increment").clicked() {
                let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new("/rdata/get_state");
                self.ws.add_ws_write(serde_json::to_string(&cmd).unwrap());

                // {
                //     let args = rctrl_rosbridge::lifecycle_msgs::srv::change_state::Request::activate();
                //     let cmd = rctrl_rosbridge::protocol::CallService::new("/rdata/change_state").with_args(&args).cmd();
                //     let mut w = ws_write_lock.write().unwrap();
                //     w.push_back(serde_json::to_string(&cmd).unwrap());
                // }

                // {
                //     let args = rctrl_rosbridge::lifecycle_msgs::srv::change_state::Request::shutdown();
                //     let cmd = rctrl_rosbridge::protocol::CallService::new("/rdata/change_state").with_args(&args).cmd();
                //     let mut w = ws_write_lock.write().unwrap();
                //     w.push_back(serde_json::to_string(&cmd).unwrap());
                // }
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {});

        self.lifecycle_manager.draw(ctx);
    }
}
