use crate::gui::{gui_elem::gen_gui_elem_id, gui_elem::GuiElem};
use crate::ws_lock::WsLock;
use eframe::egui;
use gloo_console::log;
use rctrl_rosbridge::recu_msgs::msg::ValveState;
use serde_json::Value;
use std::rc::Rc;
use std::sync::Mutex;

pub struct ValveControl {
    ws_lock: Rc<WsLock>,
    valve_name: String,
    state_display: Rc<Mutex<ValveStateDisplay>>,
    display_name: String,
}

impl ValveControl {
    pub fn new(ws_lock: &Rc<WsLock>, valve_name: String, display_name: String) -> Self {
        Self {
            ws_lock: Rc::clone(&ws_lock),
            valve_name: valve_name.clone(),
            state_display: ValveStateDisplay::new_shared(&ws_lock, valve_name),
            display_name: display_name,
        }
    }

    pub fn draw(&self, ctx: &egui::Context, ui: &mut egui::Ui) {
        ui.group(|ui| {
            ui.set_width(70.0);
            ui.vertical_centered_justified(|ui| {
                ui.label(self.display_name.clone());

                let mut state_display_lock = self.state_display.lock().unwrap();
                state_display_lock.draw(ui);

                match state_display_lock.state {
                    ValveState::Closed => {
                        if ui.button("Open").clicked() {
                            let service = format!("/{}/open", self.valve_name.to_owned());
                            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&service);
                            self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

                            let service = format!("/{}/get_state", self.valve_name.to_owned());
                            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&service);
                            self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
                        }
                    }
                    ValveState::Open => {
                        if ui.button("Close").clicked() {
                            let service = format!("/{}/close", self.valve_name.to_owned());
                            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&service);
                            self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

                            let service = format!("/{}/get_state", self.valve_name.to_owned());
                            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&service);
                            self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
                        }
                    }
                    ValveState::Unknown => {
                        if ui.button("🔄").clicked() {
                            let service = format!("/{}/get_state", self.valve_name.to_owned());
                            let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&service);
                            self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
                        }
                    }
                }
            });
        });
    }

    pub fn deregister(&self) {
        // deregister children
    }
}

struct ValveStateDisplay {
    id: u32,
    op: String,
    topic: String,
    ws_lock: Rc<WsLock>,
    pub state: ValveState,
}

impl ValveStateDisplay {
    pub fn new_shared(ws_lock: &Rc<WsLock>, valve_name: String) -> Rc<Mutex<Self>> {
        let service = format!("/recu/{}/get_state", valve_name.to_owned());

        let valve_state_display = Self {
            id: gen_gui_elem_id(),
            op: ("service_response").to_owned(),
            topic: (service).to_owned(),
            ws_lock: Rc::clone(&ws_lock),
            state: ValveState::Unknown,
        };

        let op = valve_state_display.op.clone();
        let topic = valve_state_display.topic.clone();
        let id = valve_state_display.id.clone();
        let valve_state_display_lock = Rc::new(Mutex::new(valve_state_display));
        let valve_state_display_lock_c = Rc::clone(&valve_state_display_lock);
        ws_lock.add_gui_elem(op, topic, id, valve_state_display_lock_c);

        return valve_state_display_lock;
    }
}

impl GuiElem for ValveStateDisplay {
    // Most of this is ripped from the reference
    // REFERENCE: https://docs.rs/egui/latest/src/egui/widgets/selected_label.rs.html#25-28
    fn draw(&mut self, ui: &mut egui::Ui) {
        let mut widget_text = egui::WidgetText::from(egui::RichText::new("Unknown"));

        if self.state == ValveState::Open {
            widget_text = egui::WidgetText::from(egui::RichText::new("Opened"));
        } else if self.state == ValveState::Closed {
            widget_text = egui::WidgetText::from(egui::RichText::new("Closed"));
        }

        let button_padding = ui.spacing().button_padding;
        let total_extra = button_padding + button_padding;
        let wrap_width = ui.available_width() - total_extra.x;
        let text = widget_text.into_galley(ui, None, wrap_width, egui::TextStyle::Button);

        let desired_size = eframe::egui::Vec2::new(ui.available_width(), ui.spacing().interact_size.y);
        let (rect, response) = ui.allocate_at_least(desired_size, egui::Sense::hover());

        if ui.is_rect_visible(response.rect) {
            let text_pos = ui.layout().align_size_within_rect(text.size(), rect.shrink2(button_padding)).min;

            let visuals = ui.style().interact_selectable(&response, false);

            let rect = rect.expand(visuals.expansion);
            let mut bg_color = egui::Color32::from_gray(10);
            let mut text_color = visuals.text_color();
            let outline_stroke = egui::Stroke::new(0.0, egui::Color32::BLACK);
            if self.state == ValveState::Open {
                bg_color = egui::Color32::from_rgb(27, 131, 4);
                text_color = egui::Color32::BLACK;
            } else if self.state == ValveState::Closed {
                bg_color = egui::Color32::from_rgb(131, 44, 4);
                text_color = egui::Color32::BLACK;
            }

            ui.painter().rect(rect, visuals.rounding, bg_color, outline_stroke);
            text.paint_with_color_override(ui.painter(), text_pos, text_color);
        }
    }

    fn update_data(&mut self, data: &Value) {
        match serde_json::from_value::<rctrl_rosbridge::recu_msgs::srv::get_valve_state::Response>(data.clone()) {
            Ok(values) => self.state = values.current_state,
            Err(e) => {
                self.state = ValveState::Unknown;
                log!(format!("ValveStateDisplay::update_data() failed: {}", e));
            }
        }
    }

    fn deregister(&self) {
        self.ws_lock.remove_gui_elem(self.op.clone(), self.topic.clone(), self.id.clone())
    }
}
