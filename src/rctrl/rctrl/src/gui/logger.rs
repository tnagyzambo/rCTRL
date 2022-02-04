use crate::gui::gui_elem::GuiElem;
use crate::ws_lock::WsLock;
use eframe::egui;
use gloo_console::log;
use serde_json::Value;
use std::rc::Rc;
use std::sync::Mutex;

/// Main [`Logger`] structure.
pub struct Logger {
    ws_lock: Rc<WsLock>,
    pub open: bool,
    logs: Vec<String>,
}

impl Logger {
    pub fn new_shared(ws_lock: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let logger = Self {
            ws_lock: Rc::clone(&ws_lock),
            open: true,
            logs: Vec::new(),
        };

        let op = ("publish").to_owned();
        let topic = ("/rosout").to_owned();
        let logger_lock = Rc::new(Mutex::new(logger));
        let logger_lock_c = Rc::clone(&logger_lock);
        ws_lock.add_gui_elem(op, topic, logger_lock_c);

        return logger_lock;
    }
}

impl GuiElem for Logger {
    fn draw(&self, ui: &mut egui::Ui) {
        let text_style = egui::TextStyle::Body;
        let row_height = ui.fonts()[text_style].row_height();
        egui::ScrollArea::vertical()
            .stick_to_bottom()
            .show_rows(ui, row_height, self.logs.len(), |ui, row_range| {
                for row in row_range {
                    ui.label(&self.logs[row]);
                }
            });

        ui.ctx().request_repaint();
    }

    fn update_data(&mut self, data: Value) {
        match serde_json::from_value::<rctrl_rosbridge::rosout::msg::log::Log>(data) {
            Ok(values) => self.logs.push(values.msg),
            Err(e) => {
                log!(format!("Logger::update_data() failed: {}", e));
            }
        }
    }
}
