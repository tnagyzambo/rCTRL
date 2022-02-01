use crate::lc::LifecycleManager;
use crate::{GuiElem, GuiElems};
use eframe::{egui, epi};
use std::collections::VecDeque;
use std::rc::Rc;
use std::sync::RwLock;

/// We derive Deserialize/Serialize so we can persist app state on shutdown.
#[cfg_attr(feature = "persistence", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "persistence", serde(default))] // if we add new fields, give them default values when deserializing old state
pub struct Gui {
    ws_read_lock: Rc<RwLock<GuiElems<String, String>>>,
    ws_write_lock: Rc<RwLock<VecDeque<String>>>,
    lifecycle_manager: LifecycleManager,
}

impl Gui {
    pub fn new(
        ws_read_lock: Rc<RwLock<GuiElems<String, String>>>,
        ws_write_lock: Rc<RwLock<VecDeque<String>>>,
    ) -> Self {
        Self {
            ws_read_lock: ws_read_lock,
            ws_write_lock: ws_write_lock,
            lifecycle_manager: LifecycleManager::default(),
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
        self.lifecycle_manager.connect_to_ws(&self.ws_read_lock);
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
            ws_read_lock,
            ws_write_lock,
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
                {
                    let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new("/rdata/get_state");
                    let mut w = ws_write_lock.write().unwrap();
                    w.push_back(serde_json::to_string(&cmd).unwrap());
                }

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

        self.lifecycle_manager
            .draw(ctx, &self.ws_read_lock, &self.ws_write_lock);
    }
}
