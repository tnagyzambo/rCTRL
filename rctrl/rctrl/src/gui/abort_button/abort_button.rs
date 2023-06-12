use crate::gui::{gui_elem::gen_gui_elem_id, gui_elem::GuiElem};
use crate::ws_lock::WsLock;
use eframe::egui;
use gloo_console::log;
use serde_json::Value;
use std::rc::Rc;
use std::sync::Mutex;

pub struct AbortButton {
    ws_lock: Rc<WsLock>,
}

impl AbortButton {
    pub fn new(ws_lock: &Rc<WsLock>) -> Self {
        Self {
            ws_lock: Rc::clone(&ws_lock),
        }
    }

    pub fn draw(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        let widget_text = egui::WidgetText::from(egui::RichText::new("ABORT"));

        let button_padding = ui.spacing().button_padding;
        let total_extra = button_padding + button_padding;
        let wrap_width = ui.available_width() - total_extra.x;
        let text = widget_text.into_galley(ui, None, wrap_width, egui::TextStyle::Button);

        let desired_size = eframe::egui::Vec2::new(100.0, 50.0);
        let (rect, mut response) = ui.allocate_at_least(desired_size, egui::Sense::click());

        let text_pos = ui.layout().align_size_within_rect(text.size(), rect.shrink2(button_padding)).min;

        let visuals = ui.style().interact_selectable(&response, false);

        let rect = rect.expand(visuals.expansion);
        let bg_color = egui::Color32::from_rgb(131, 44, 4);
        let text_color = egui::Color32::BLACK;
        let outline_stroke = egui::Stroke::new(0.0, egui::Color32::BLACK);

        ui.painter().rect(rect, visuals.rounding, bg_color, outline_stroke);
        text.paint_with_color_override(ui.painter(), text_pos, text_color);

        if response.clicked() {
            let service = "/recu/abort";
            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&service);
            self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
        }

        let events = ui.input().events.clone();
        for event in &events {
            match event {
                egui::Event::Key{key, pressed, modifiers} => {
                    match key {
                        egui::Key::Space => {
                            if *pressed {
                                let service = "/recu/abort";
                                let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&service);
                                self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
                            }
                        },
                        _ => {},
                    }
                },
                egui::Event::Text(t) => { println!("Text = {:?}", t) }
                _ => {}
            }
        }
    }
}
