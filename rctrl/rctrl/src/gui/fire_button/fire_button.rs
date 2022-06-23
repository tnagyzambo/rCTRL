use crate::gui::{gui_elem::gen_gui_elem_id, gui_elem::GuiElem};
use crate::ws_lock::WsLock;
use eframe::egui;
use gloo_console::log;
use serde_json::Value;
use std::rc::Rc;
use std::sync::Mutex;

pub struct FireButton {
    confirm_window: ConfirmWindow,
}

impl FireButton {
    pub fn new(ws_lock: &Rc<WsLock>) -> Self {
        Self {
            confirm_window: ConfirmWindow::new(ws_lock, format!("Fire Sequence")),
        }
    }

    pub fn draw(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        match &self.confirm_window.open {
            true => self.confirm_window.draw(ctx, ui),
            false => (),
        };

        // if ui.button("FIRE").clicked() {
        //     self.confirm_window.open = true;
        // }

        let widget_text = egui::WidgetText::from(egui::RichText::new("FIRE"));

        let button_padding = ui.spacing().button_padding;
        let total_extra = button_padding + button_padding;
        let wrap_width = ui.available_width() - total_extra.x;
        let text = widget_text.into_galley(ui, None, wrap_width, egui::TextStyle::Button);

        let desired_size = eframe::egui::Vec2::new(100.0, 50.0);
        let (rect, mut response) = ui.allocate_at_least(desired_size, egui::Sense::click());

        if ui.is_rect_visible(response.rect) {
            let text_pos = ui.layout().align_size_within_rect(text.size(), rect.shrink2(button_padding)).min;

            let visuals = ui.style().interact_selectable(&response, false);

            let rect = rect.expand(visuals.expansion);
            let bg_color = egui::Color32::from_rgb(27, 131, 4);
            let text_color = egui::Color32::BLACK;
            let outline_stroke = egui::Stroke::new(0.0, egui::Color32::BLACK);

            ui.painter().rect(rect, visuals.rounding, bg_color, outline_stroke);
            text.paint_with_color_override(ui.painter(), text_pos, text_color);
        }

        if response.clicked() {
            self.confirm_window.open = true;
            response.mark_changed();
        }
    }
}

struct ConfirmWindow {
    ws_lock: Rc<WsLock>,
    title: String,
    open: bool,
    confirm_button: Rc<Mutex<ConfirmButton>>,
}

impl ConfirmWindow {
    pub fn new(ws_lock: &Rc<WsLock>, title: String) -> Self {
        Self {
            ws_lock: Rc::clone(&ws_lock),
            title: title,
            open: false,
            confirm_button: ConfirmButton::new_shared(ws_lock),
        }
    }

    fn draw(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        let cmd_complete = false;

        egui::Window::new(self.title.clone())
            .collapsible(false)
            .resizable(false)
            .default_pos(egui::Pos2::new(300.0, 350.0))
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    let cmd_complete = self.confirm_button.lock().unwrap().cmd_complete;
                    if !cmd_complete {
                        ui.columns(2, |columns| {
                            columns[0].centered_and_justified(|ui| {
                                self.confirm_button.lock().unwrap().draw(ui);
                            });
                            columns[1].centered_and_justified(|ui| {
                                if ui.button("Cancel").clicked() {
                                    self.open = false;
                                };
                            });
                        });
                    } else {
                        ui.centered_and_justified(|ui| {
                            if ui.button("Close").clicked() {
                                self.open = false;
                                self.confirm_button.lock().unwrap().cmd_complete = false;
                            };
                        });
                    }
                });
            });
    }
}

impl Drop for ConfirmWindow {
    fn drop(&mut self) {
        self.confirm_button.lock().unwrap().deregister();
    }
}

struct ConfirmButton {
    id: u32,
    op: String,
    topic: String,
    ws_lock: Rc<WsLock>,
    cmd_complete: bool,
}

impl ConfirmButton {
    pub fn new_shared(ws_lock: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let rstate_action_panel = Self {
            id: gen_gui_elem_id(),
            op: ("service_response").to_owned(),
            topic: ("/rstate/change_state").to_owned(),
            ws_lock: Rc::clone(&ws_lock),
            cmd_complete: false,
        };

        let op = rstate_action_panel.op.clone();
        let topic = rstate_action_panel.topic.clone();
        let id = rstate_action_panel.id.clone();
        let rstate_action_panel_lock = Rc::new(Mutex::new(rstate_action_panel));
        let rstate_action_panel_lock_c = Rc::clone(&rstate_action_panel_lock);
        ws_lock.add_gui_elem(op, topic, id, rstate_action_panel_lock_c);

        return rstate_action_panel_lock;
    }
}

impl GuiElem for ConfirmButton {
    fn draw(&mut self, ui: &mut egui::Ui) {
        if ui.button("Confirm").clicked() {
            let service = "/recu/fire";
            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&service);
            self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
            self.cmd_complete = true;
        }
    }

    fn update_data(&mut self, data: &Value) {
        // match serde_json::from_value::<rctrl_rosbridge::lifecycle_msgs::srv::change_state::Response>(data.clone()) {
        //     Ok(values) => match values.success {
        //         true => self.cmd_complete = true,
        //         false => {
        //             self.cmd_complete = true;
        //             log!("ERROR CHANGING RSTATE LC STATE");
        //         }
        //     },
        //     Err(e) => {
        //         log!(format!("RStateLcWindowConfirm::update_data() failed: {}", e));
        //     }
        // }
    }

    fn deregister(&self) {
        self.ws_lock.remove_gui_elem(self.op.clone(), self.topic.clone(), self.id.clone())
    }
}
