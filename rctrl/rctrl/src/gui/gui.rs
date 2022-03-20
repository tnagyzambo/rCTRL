use crate::ws_lock::WsLock;
use eframe::{egui, epi};
use std::rc::Rc;
use std::sync::Mutex;

use crate::gui::GuiElem;
use crate::gui::LifecycleManager;
use crate::gui::Logger;
use crate::gui::Rstate;

// Extend epi:App trait
pub trait App: epi::App {
    // Function to call when switching between apps to refresh any data needed
    fn refresh(&mut self);
}

pub struct Apps {
    lifecycle_manager: LifecycleManager,
    rstate: Rstate,
}

impl Apps {
    fn new(ws_lock: &Rc<WsLock>) -> Self {
        Self {
            lifecycle_manager: LifecycleManager::new(&Rc::clone(&ws_lock)),
            rstate: Rstate::new(&Rc::clone(&ws_lock)),
        }
    }

    fn iter_mut(&mut self) -> impl Iterator<Item = (&str, &str, &mut dyn App)> {
        vec![
            (
                "Lifecycle Manager",
                "lifecycle_manager",
                &mut self.lifecycle_manager as &mut dyn App,
            ),
            ("rSTATE", "rstate", &mut self.rstate as &mut dyn App),
        ]
        .into_iter()
    }
}

/// Main Gui object.
pub struct Gui {
    ws_lock: Rc<WsLock>,
    selected_anchor: String,
    apps: Apps,
    logger: Rc<Mutex<Logger>>,
}

impl Gui {
    pub fn new(ws_lock: &Rc<WsLock>) -> Self {
        Self {
            ws_lock: Rc::clone(&ws_lock),
            selected_anchor: "none".to_string(),
            apps: Apps::new(&ws_lock),
            logger: Logger::new_shared(&ws_lock),
        }
    }
}

impl epi::App for Gui {
    fn name(&self) -> &str {
        "rctrl"
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
            selected_anchor,
            apps,
            logger,
        } = self;

        egui::SidePanel::left("Apps").show(ctx, |ui| {
            for (name, anchor, app) in self.apps.iter_mut() {
                if ui.selectable_label(self.selected_anchor == anchor, name).clicked() {
                    self.selected_anchor = anchor.to_owned();
                    app.refresh();
                    if frame.is_web() {
                        ui.output().open_url(format!("#{}", anchor));
                    }
                }
            }
        });

        for (_name, anchor, app) in self.apps.iter_mut() {
            if anchor == self.selected_anchor || ctx.memory().everything_is_visible() {
                app.update(ctx, frame);
            }
        }

        {
            // Need to explicitly split up 2the borrowing here or else the borrow checker will complain about
            // borrowing an already mutable reference to self
            // REFERENCE: <https://doc.rust-lang.org/nomicon/borrow-splitting.html>
            let logger = self.logger.lock().unwrap();
            egui::TopBottomPanel::bottom("Logger")
                .resizable(true)
                .default_height(200.0)
                .show(&ctx, |ui| logger.draw(ui));
        }
    }
}
