use reqwasm::websocket::{Message, futures::WebSocket};
use gloo_console::log;
use wasm_bindgen_futures::spawn_local;
use futures::{SinkExt, StreamExt};
use futures::stream::{SplitSink, SplitStream};
use std::rc::Rc;
use std::sync::RwLock;
use std::collections::HashMap;
use std::any::Any;

use eframe::{egui, epi};

use crate::rosbridge;

pub trait MyTrait {
    fn update(&self, ctx: &egui::CtxRef, frame: &epi::Frame);
    fn up(&mut self, value: f32);
  }

pub struct GuiThing {
    pub value: f32,
}

pub struct GuiThot {
    pub value: String,
}

impl GuiThing {
    pub fn new() -> Self {
        Self {
            value: 6.0,
        }
    }
}

impl MyTrait for GuiThing {
    fn update(&self, ctx: &egui::CtxRef, frame: &epi::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            // The top panel is often a good place for a menu bar:
            egui::menu::bar(ui, |ui| {
                ui.label(format!("{}", self.value));
            });
        });
    }

    fn up(&mut self, value: f32) {
        self.value = value;
    }
}

impl GuiThot {
    pub fn new() -> Self {
        Self {
            value: String::from("sheeeet"),
        }
    }
}

impl MyTrait for GuiThot {
    fn update(&self, ctx: &egui::CtxRef, frame: &epi::Frame) {
    }
    fn up(&mut self, value: f32) {
    }
}


/// We derive Deserialize/Serialize so we can persist app state on shutdown.
#[cfg_attr(feature = "persistence", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "persistence", serde(default))] // if we add new fields, give them default values when deserializing old state
pub struct RctrlGUI {
    label: String,
    ws_read_lock: Rc<RwLock<HashMap<String, Box<MyTrait>>>>,
    ws_write_lock: Rc<RwLock<Vec<String>>>,

    // this how you opt-out of serialization of a member
    #[cfg_attr(feature = "persistence", serde(skip))]
    pub value: f32,
}

impl RctrlGUI {
    pub fn new(ws_read_lock: Rc<RwLock<HashMap<String, Box<MyTrait>>>>, ws_write_lock: Rc<RwLock<Vec<String>>>) -> Self {
        Self {
            label: "Hello World!".to_owned(),
            ws_read_lock: ws_read_lock,
            ws_write_lock: ws_write_lock,
            value: 2.7,
        }
    }
}

impl epi::App for RctrlGUI {
    fn name(&self) -> &str {
        "eframe template"
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
        let Self { label, value, ws_read_lock, ws_write_lock } = self;

        // Examples of how to create different panels and windows.
        // Pick whichever suits you.
        // Tip: a good default choice is to just keep the `CentralPanel`.
        // For inspiration and more examples, go to https://emilk.github.io/egui

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

            ui.horizontal(|ui| {
                ui.label("Write something: ");
                ui.text_edit_singleline(label);
            });

            ui.add(egui::Slider::new(value, 0.0..=10.0).text("value"));
            if ui.button("Increment").clicked() {
                

                //*value = gui_things.value;

                {
                    let mut w = ws_write_lock.write().unwrap();
                    w.push(String::from("6.0"));
                }
            }

            ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 0.0;
                    ui.label("powered by ");
                    ui.hyperlink_to("egui", "https://github.com/emilk/egui");
                    ui.label(" and ");
                    ui.hyperlink_to("eframe", "https://github.com/emilk/egui/tree/master/eframe");
                });
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            // The central panel the region left after adding TopPanel's and SidePanel's

            ui.heading("eframe template");
            ui.hyperlink("https://github.com/emilk/eframe_template");
            ui.add(egui::github_link_file!(
                "https://github.com/emilk/eframe_template/blob/master/",
                "Source code."
            ));
            egui::warn_if_debug_build(ui);
        });

        if false {
            egui::Window::new("Window").show(ctx, |ui| {
                ui.label("Windows can be moved by dragging them.");
                ui.label("They are automatically sized based on contents.");
                ui.label("You can turn on resizing and scrolling if you like.");
                ui.label("You would normally chose either panels OR windows.");
            });
        }

        let hash_map = ws_read_lock.read().unwrap();

        for gui_elem in hash_map.values() {
            gui_elem.update(ctx, frame);
        }
    }
}