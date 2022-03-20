use crate::gui::{gui_elem::GuiElem, App};
use crate::ws_lock::WsLock;
use eframe::{egui, epi};
use gloo_console::log;
use rctrl_rosbridge::lifecycle_msgs::msg::{State, Transition, TransitionDescription};
use serde_json::Value;
use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Mutex;
use web_sys::{Performance, Window};

/// Main [`LifecycleManager`] structure.
pub struct Rstate {
    ws_lock: Rc<WsLock>,
}

impl Rstate {
    pub fn new(ws_lock: &Rc<WsLock>) -> Self {
        Self {
            ws_lock: Rc::clone(&ws_lock),
        }
    }
}

impl epi::App for Rstate {
    fn name(&self) -> &str {
        "rstate"
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
        let Self { ws_lock } = self;
    }
}

impl App for Rstate {
    fn refresh(&mut self) {}
}
