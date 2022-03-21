use super::logger_hl::{highlight, CodeTheme};
use crate::gui::gui_elem::{gen_gui_elem_id, GuiElem};
use crate::ws_lock::WsLock;
use eframe::egui;
use gloo_console::log;
use serde_json::Value;
use std::rc::Rc;
use std::sync::Mutex;

/// Main [`Logger`] structure.
pub struct Logger {
    id: u32,
    ws_lock: Rc<WsLock>,
    logs: Vec<rctrl_rosbridge::rosout::msg::log::Log>,
}

impl Logger {
    pub fn new_shared(ws_lock: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let logger = Self {
            id: gen_gui_elem_id(),
            ws_lock: Rc::clone(&ws_lock),
            logs: Vec::new(),
        };

        let op = ("publish").to_owned();
        let topic = ("/rosout").to_owned();
        let id = logger.id;
        let logger_lock = Rc::new(Mutex::new(logger));
        let logger_lock_c = Rc::clone(&logger_lock);
        ws_lock.add_gui_elem(op, topic, id, logger_lock_c);

        return logger_lock;
    }
}

impl GuiElem for Logger {
    fn draw(&self, ui: &mut egui::Ui) {
        let theme = CodeTheme::from_memory(ui.ctx());

        let layouter = |ui: &egui::Ui, string: &str| {
            let layout_job = highlight(ui.ctx(), &theme, string);
            ui.fonts().layout_job(layout_job)
        };

        let text_style = egui::TextStyle::Body;
        let row_height = ui.text_style_height(&text_style);
        egui::ScrollArea::vertical()
            .auto_shrink([false; 2])
            .hscroll(true)
            .always_show_scroll(true)
            .stick_to_bottom()
            .show_rows(ui, row_height, self.logs.len(), |ui, row_range| {
                for row in row_range {
                    ui.horizontal(|ui| {
                        ui.set_max_height(row_height);
                        ui.set_min_height(row_height);

                        let mut log = String::new();
                        if true {
                            log.push_str("[");
                            log.push_str(&format!("{:?}", self.logs[row].level));
                            log.push_str("] ");
                        };

                        if true {
                            log.push_str("[");
                            log.push_str(&self.logs[row].stamp.sec.to_string());
                            log.push_str(".");
                            log.push_str(&self.logs[row].stamp.nanosec.to_string());
                            log.push_str("] ");
                        };

                        if true {
                            log.push_str("[");
                            log.push_str(&self.logs[row].name.to_string());
                            log.push_str("]:");
                        };

                        ui.add(egui::Label::new(layouter(&ui, &log)));

                        // Seperate widget to allow linebreaks in log messages

                        let mut log = String::new();

                        if true {
                            log.push_str(&self.logs[row].msg);
                            log.push_str(" ");
                        };

                        ui.add(egui::Label::new(layouter(&ui, &log)));
                    });
                }
            });
        ui.ctx().request_repaint();
    }

    fn update_data(&mut self, data: &Value) {
        match serde_json::from_value::<rctrl_rosbridge::rosout::msg::log::Log>(data.clone()) {
            Ok(values) => {
                self.logs.push(values);
            }
            Err(e) => {
                log!(format!("Logger::update_data() failed: {}", e));
            }
        }
    }
}
