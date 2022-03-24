use crate::gui::{gui_elem::gen_gui_elem_id, gui_elem::GuiElem, App};
use crate::ws_lock::WsLock;
use eframe::egui;
use gloo_console::log;
use rctrl_rosbridge::lifecycle_msgs::msg::{State, Transition, TransitionDescription};
use rctrl_rosbridge::rstate::msg::{NetworkState, NetworkTransition, NetworkTransitionDescription};
use serde_json::Value;
use std::rc::Rc;
use std::sync::Mutex;

pub struct RStatePanel {
    ws_lock: Rc<WsLock>,
    rstate_lc_panel: Rc<Mutex<RStateLcPanel>>,
    rstate_lc_state: Rc<Mutex<RStateLcStateDisplay>>,
    rstate_lc_transitions: Rc<Mutex<RStateLcTransitions>>,
    rstate_network_panel: Rc<Mutex<RStateNetworkPanel>>,
    rstate_network_state: Rc<Mutex<RStateNetworkStateDisplay>>,
    rstate_network_transitions: Rc<Mutex<RStateNetworkTransitions>>,
}

impl RStatePanel {
    pub fn new(ws_lock: &Rc<WsLock>) -> Self {
        Self {
            ws_lock: Rc::clone(&ws_lock),
            rstate_lc_panel: RStateLcPanel::new_shared(ws_lock),
            rstate_lc_state: RStateLcStateDisplay::new_shared(ws_lock),
            rstate_lc_transitions: RStateLcTransitions::new_shared(ws_lock),
            rstate_network_panel: RStateNetworkPanel::new_shared(ws_lock),
            rstate_network_state: RStateNetworkStateDisplay::new_shared(ws_lock),
            rstate_network_transitions: RStateNetworkTransitions::new_shared(ws_lock),
        }
    }

    pub fn draw(&self, ui: &mut egui::Ui) {
        let text_width = 120.0;
        let state_width = 100.0;
        ui.group(|ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    ui.scope(|ui| {
                        ui.set_width(text_width);
                        self.rstate_lc_panel.lock().unwrap().draw(ui);
                    });
                    ui.scope(|ui| {
                        ui.set_width(state_width);
                        self.rstate_lc_state.lock().unwrap().draw(ui);
                    });
                    self.rstate_lc_transitions.lock().unwrap().draw(ui);
                });
                ui.horizontal(|ui| {
                    ui.scope(|ui| {
                        ui.set_width(text_width);
                        self.rstate_network_panel.lock().unwrap().draw(ui);
                    });
                    ui.scope(|ui| {
                        ui.set_width(state_width);
                        self.rstate_network_state.lock().unwrap().draw(ui);
                    });
                    self.rstate_network_transitions.lock().unwrap().draw(ui);
                });
            });
        });
    }

    pub fn deregister(&self) {
        // deregister children
    }
}

struct RStateLcPanel {
    id: u32,
    op: String,
    topic: String,
    ws_lock: Rc<WsLock>,
}

impl RStateLcPanel {
    pub fn new_shared(ws_lock: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let rstate_lc_panel = Self {
            id: gen_gui_elem_id(),
            op: ("publish").to_owned(),
            topic: ("/rstate/transition_event").to_owned(),
            ws_lock: Rc::clone(&ws_lock),
        };

        let cmd = rctrl_rosbridge::protocol::Subscribe::new(&rstate_lc_panel.topic);
        ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

        let op = rstate_lc_panel.op.clone();
        let topic = rstate_lc_panel.topic.clone();
        let id = rstate_lc_panel.id.clone();
        let rstate_lc_panel_lock = Rc::new(Mutex::new(rstate_lc_panel));
        let rstate_lc_panel_lock_c = Rc::clone(&rstate_lc_panel_lock);
        ws_lock.add_gui_elem(op, topic, id, rstate_lc_panel_lock_c);

        return rstate_lc_panel_lock;
    }
}

impl GuiElem for RStateLcPanel {
    fn draw(&self, ui: &mut egui::Ui) {
        ui.with_layout(egui::Layout::right_to_left(), |ui| {
            ui.label("rSTATE Node Status:");
        });
    }

    fn update_data(&mut self, data: &Value) {
        // Request state of node
        let topic = "/rstate/get_state";
        let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
        self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

        // Request available transitions of node
        let topic = "/rstate/get_available_transitions";
        let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
        self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
    }

    fn deregister(&self) {
        self.ws_lock.remove_gui_elem(self.op.clone(), self.topic.clone(), self.id.clone())
    }
}

struct RStateLcStateDisplay {
    id: u32,
    op: String,
    topic: String,
    ws_lock: Rc<WsLock>,
    pub state: State,
}

impl RStateLcStateDisplay {
    pub fn new_shared(ws_lock: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let rstate_lc_state_display = Self {
            id: gen_gui_elem_id(),
            op: ("service_response").to_owned(),
            topic: ("/rstate/get_state").to_owned(),
            ws_lock: Rc::clone(&ws_lock),
            state: State::Unknown,
        };

        let op = rstate_lc_state_display.op.clone();
        let topic = rstate_lc_state_display.topic.clone();
        let id = rstate_lc_state_display.id.clone();
        let rstate_lc_state_display_lock = Rc::new(Mutex::new(rstate_lc_state_display));
        let rstate_lc_state_display_lock_c = Rc::clone(&rstate_lc_state_display_lock);
        ws_lock.add_gui_elem(op, topic, id, rstate_lc_state_display_lock_c);

        return rstate_lc_state_display_lock;
    }
}

impl GuiElem for RStateLcStateDisplay {
    // Most of this is ripped from the reference
    // REFERENCE: https://docs.rs/egui/latest/src/egui/widgets/selected_label.rs.html#25-28
    fn draw(&self, ui: &mut egui::Ui) {
        let widget_text = egui::WidgetText::from(egui::RichText::new(format!("{:?}", self.state)));

        let button_padding = ui.spacing().button_padding;
        let total_extra = button_padding + button_padding;
        let wrap_width = ui.available_width() - total_extra.x;
        let text = widget_text.into_galley(ui, None, wrap_width, egui::TextStyle::Button);

        let mut desired_size = eframe::egui::Vec2::new(ui.available_width(), ui.spacing().interact_size.y);
        let (rect, response) = ui.allocate_at_least(desired_size, egui::Sense::hover());

        if ui.is_rect_visible(response.rect) {
            let text_pos = ui.layout().align_size_within_rect(text.size(), rect.shrink2(button_padding)).min;

            let visuals = ui.style().interact_selectable(&response, false);

            let rect = rect.expand(visuals.expansion);
            let mut bg_color = egui::Color32::from_gray(10);
            let mut text_color = visuals.text_color();
            let outline_stroke = egui::Stroke::new(0.0, egui::Color32::BLACK);
            if self.state == State::Active {
                bg_color = egui::Color32::from_rgb(27, 131, 4);
                text_color = egui::Color32::BLACK;
            } else if self.state == State::Unknown
                || self.state == State::ErrorProcessing
                || self.state == State::ShuttingDown
                || self.state == State::Finalized
            {
                bg_color = egui::Color32::from_rgb(131, 44, 4);
                text_color = egui::Color32::BLACK;
            }

            ui.painter().rect(rect, visuals.rounding, bg_color, outline_stroke);
            text.paint_with_color_override(ui.painter(), text_pos, text_color);
        }
    }

    fn update_data(&mut self, data: &Value) {
        match serde_json::from_value::<rctrl_rosbridge::lifecycle_msgs::srv::get_state::Response>(data.clone()) {
            Ok(values) => self.state = values.current_state,
            Err(e) => {
                self.state = State::Unknown;
                log!(format!("RStateLCStateDisplay::update_data() failed: {}", e));
            }
        }
    }

    fn deregister(&self) {
        self.ws_lock.remove_gui_elem(self.op.clone(), self.topic.clone(), self.id.clone())
    }
}

pub struct RStateLcTransitions {
    id: u32,
    op: String,
    topic: String,
    ws_lock: Rc<WsLock>,
    topic_change_state: String,
    available_transitions: Vec<TransitionDescription>,
}

impl RStateLcTransitions {
    pub fn new_shared(ws_lock: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let rstate_lc_transitions = Self {
            id: gen_gui_elem_id(),
            op: ("service_response").to_owned(),
            topic: ("/rstate/get_available_transitions").to_owned(),
            ws_lock: Rc::clone(&ws_lock),
            topic_change_state: ("/rstate/change_state").to_owned(),
            available_transitions: Vec::new(),
        };

        let op = rstate_lc_transitions.op.clone();
        let topic = rstate_lc_transitions.topic.clone();
        let id = rstate_lc_transitions.id.clone();
        let rstate_lc_transitions_lock = Rc::new(Mutex::new(rstate_lc_transitions));
        let rstate_lc_transitions_lock_c = Rc::clone(&rstate_lc_transitions_lock);
        ws_lock.add_gui_elem(op, topic, id, rstate_lc_transitions_lock_c);

        return rstate_lc_transitions_lock;
    }

    fn is_enabled(&self, transition: &Transition) -> bool {
        for elem in &self.available_transitions {
            if &elem.transition == transition {
                return true;
            }
        }
        false
    }

    fn draw_state_change_button(&self, ui: &mut egui::Ui, is_enabled: bool, transition: Transition, label: &str) {
        if ui.add_enabled(is_enabled, egui::Button::new(label)).clicked() {
            let args = rctrl_rosbridge::lifecycle_msgs::srv::change_state::Request::from(transition);
            let cmd = rctrl_rosbridge::protocol::CallService::new(&self.topic_change_state)
                .with_args(&args)
                .cmd();
            self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
        }
    }

    // I dont think there is ever a scenario where self.available_transitions can hold multiple
    // shutdown transition so we will just grab the first one in the Vec
    // There is a slim chance this can cause issues in the future but I doubt it
    fn get_shutdown_type(&self) -> Option<Transition> {
        for elem in &self.available_transitions {
            if elem.transition == Transition::UnconfiguredShutdown
                || elem.transition == Transition::InactiveShutdown
                || elem.transition == Transition::ActiveShutdown
            {
                return Some(elem.transition.clone());
            }
        }
        None
    }

    // The shutdown button must be treated differently since there are 3 different kinds of shutdowns
    fn draw_shutdown_button(&self, ui: &mut egui::Ui) {
        match self.get_shutdown_type() {
            Some(transition) => self.draw_state_change_button(ui, true, transition, "Shutdown"),
            None => if ui.add_enabled(false, egui::Button::new("Shutdown")).clicked() {},
        }
    }
}

impl GuiElem for RStateLcTransitions {
    fn draw(&self, ui: &mut egui::Ui) {
        ui.menu_button("▶", |ui| {
            self.draw_state_change_button(ui, self.is_enabled(&Transition::Configure), Transition::Configure, "Configure");
            self.draw_state_change_button(ui, self.is_enabled(&Transition::CleanUp), Transition::CleanUp, "Clean Up");
            self.draw_state_change_button(ui, self.is_enabled(&Transition::Activate), Transition::Activate, "Activate");
            self.draw_state_change_button(ui, self.is_enabled(&Transition::Deactivate), Transition::Deactivate, "Deactivate");
            self.draw_shutdown_button(ui);
        });
    }

    fn update_data(&mut self, data: &Value) {
        match serde_json::from_value::<rctrl_rosbridge::lifecycle_msgs::srv::get_available_transitions::Response>(data.clone()) {
            Ok(values) => {
                self.available_transitions = values.available_transitions;
            }
            Err(e) => {
                log!(format!("StateTransitions::update_data() for node \"\\rstate\" failed: {}", e));
            }
        }
    }

    fn deregister(&self) {
        self.ws_lock.remove_gui_elem(self.op.clone(), self.topic.clone(), self.id.clone())
    }
}

struct RStateNetworkPanel {
    id: u32,
    op: String,
    topic: String,
    ws_lock: Rc<WsLock>,
}

impl RStateNetworkPanel {
    pub fn new_shared(ws_lock: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let rstate_network_panel = Self {
            id: gen_gui_elem_id(),
            op: ("publish").to_owned(),
            topic: ("/rstate/transition_network_event").to_owned(),
            ws_lock: Rc::clone(&ws_lock),
        };

        let cmd = rctrl_rosbridge::protocol::Subscribe::new(&rstate_network_panel.topic);
        ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

        let op = rstate_network_panel.op.clone();
        let topic = rstate_network_panel.topic.clone();
        let id = rstate_network_panel.id.clone();
        let rstate_network_panel_lock = Rc::new(Mutex::new(rstate_network_panel));
        let rstate_network_panel_lock_c = Rc::clone(&rstate_network_panel_lock);
        ws_lock.add_gui_elem(op, topic, id, rstate_network_panel_lock_c);

        return rstate_network_panel_lock;
    }
}

impl GuiElem for RStateNetworkPanel {
    fn draw(&self, ui: &mut egui::Ui) {
        ui.with_layout(egui::Layout::right_to_left(), |ui| {
            ui.label("Network Status:");
        });
    }

    fn update_data(&mut self, data: &Value) {
        // Request state of node
        let topic = "/rstate/get_network_state";
        let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
        self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

        // Request available transitions of node
        let topic = "/rstate/get_available_network_transitions";
        let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
        self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
    }

    fn deregister(&self) {
        self.ws_lock.remove_gui_elem(self.op.clone(), self.topic.clone(), self.id.clone())
    }
}

struct RStateNetworkStateDisplay {
    id: u32,
    op: String,
    topic: String,
    ws_lock: Rc<WsLock>,
    pub state: NetworkState,
}

impl RStateNetworkStateDisplay {
    pub fn new_shared(ws_lock: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let rstate_network_state_display = Self {
            id: gen_gui_elem_id(),
            op: ("service_response").to_owned(),
            topic: ("/rstate/get_network_state").to_owned(),
            ws_lock: Rc::clone(&ws_lock),
            state: NetworkState::Unknown,
        };

        let op = rstate_network_state_display.op.clone();
        let topic = rstate_network_state_display.topic.clone();
        let id = rstate_network_state_display.id.clone();
        let rstate_network_state_display_lock = Rc::new(Mutex::new(rstate_network_state_display));
        let rstate_network_state_display_lock_c = Rc::clone(&rstate_network_state_display_lock);
        ws_lock.add_gui_elem(op, topic, id, rstate_network_state_display_lock_c);

        return rstate_network_state_display_lock;
    }
}

impl GuiElem for RStateNetworkStateDisplay {
    // Most of this is ripped from the reference
    // REFERENCE: https://docs.rs/egui/latest/src/egui/widgets/selected_label.rs.html#25-28
    fn draw(&self, ui: &mut egui::Ui) {
        let widget_text = egui::WidgetText::from(egui::RichText::new(format!("{:?}", self.state)));

        let button_padding = ui.spacing().button_padding;
        let total_extra = button_padding + button_padding;
        let wrap_width = ui.available_width() - total_extra.x;
        let text = widget_text.into_galley(ui, None, wrap_width, egui::TextStyle::Button);

        let mut desired_size = eframe::egui::Vec2::new(ui.available_width(), ui.spacing().interact_size.y);
        let (rect, response) = ui.allocate_at_least(desired_size, egui::Sense::hover());

        if ui.is_rect_visible(response.rect) {
            let text_pos = ui.layout().align_size_within_rect(text.size(), rect.shrink2(button_padding)).min;

            let visuals = ui.style().interact_selectable(&response, false);

            let rect = rect.expand(visuals.expansion);
            let mut bg_color = egui::Color32::from_gray(10);
            let mut text_color = visuals.text_color();
            let outline_stroke = egui::Stroke::new(0.0, egui::Color32::BLACK);
            if self.state == NetworkState::Active || self.state == NetworkState::Armed {
                bg_color = egui::Color32::from_rgb(27, 131, 4);
                text_color = egui::Color32::BLACK;
            } else if self.state == NetworkState::Unknown
                || self.state == NetworkState::ErrorProcessing
                || self.state == NetworkState::ShuttingDown
                || self.state == NetworkState::Finalized
            {
                bg_color = egui::Color32::from_rgb(131, 44, 4);
                text_color = egui::Color32::BLACK;
            }

            ui.painter().rect(rect, visuals.rounding, bg_color, outline_stroke);
            text.paint_with_color_override(ui.painter(), text_pos, text_color);
        }
    }

    fn update_data(&mut self, data: &Value) {
        match serde_json::from_value::<rctrl_rosbridge::rstate::srv::get_network_state::Response>(data.clone()) {
            Ok(values) => self.state = values.current_state,
            Err(e) => {
                self.state = NetworkState::Unknown;
                log!(format!("RStateNetworkStateDisplay::update_data() failed: {}", e));
            }
        }
    }

    fn deregister(&self) {
        self.ws_lock.remove_gui_elem(self.op.clone(), self.topic.clone(), self.id.clone())
    }
}

pub struct RStateNetworkTransitions {
    id: u32,
    op: String,
    topic: String,
    ws_lock: Rc<WsLock>,
    topic_change_state: String,
    available_transitions: Vec<NetworkTransitionDescription>,
}

impl RStateNetworkTransitions {
    pub fn new_shared(ws_lock: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let rstate_network_transitions = Self {
            id: gen_gui_elem_id(),
            op: ("service_response").to_owned(),
            topic: ("/rstate/get_available_network_transitions").to_owned(),
            ws_lock: Rc::clone(&ws_lock),
            topic_change_state: ("/rstate/change_network_state").to_owned(),
            available_transitions: Vec::new(),
        };

        let op = rstate_network_transitions.op.clone();
        let topic = rstate_network_transitions.topic.clone();
        let id = rstate_network_transitions.id.clone();
        let rstate_network_transitions_lock = Rc::new(Mutex::new(rstate_network_transitions));
        let rstate_lc_transitions_lock_c = Rc::clone(&rstate_network_transitions_lock);
        ws_lock.add_gui_elem(op, topic, id, rstate_lc_transitions_lock_c);

        return rstate_network_transitions_lock;
    }

    fn is_enabled(&self, transition: &NetworkTransition) -> bool {
        for elem in &self.available_transitions {
            if &elem.transition == transition {
                return true;
            }
        }
        false
    }

    fn draw_state_change_button(&self, ui: &mut egui::Ui, is_enabled: bool, transition: NetworkTransition, label: &str) {
        if ui.add_enabled(is_enabled, egui::Button::new(label)).clicked() {
            let args = rctrl_rosbridge::rstate::srv::change_network_state::Request::from(transition);
            let cmd = rctrl_rosbridge::protocol::CallService::new(&self.topic_change_state)
                .with_args(&args)
                .cmd();
            self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
        }
    }
}

impl GuiElem for RStateNetworkTransitions {
    fn draw(&self, ui: &mut egui::Ui) {
        ui.menu_button("▶", |ui| {
            self.draw_state_change_button(
                ui,
                self.is_enabled(&NetworkTransition::Configure),
                NetworkTransition::Configure,
                "Configure",
            );
            self.draw_state_change_button(
                ui,
                self.is_enabled(&NetworkTransition::CleanUp),
                NetworkTransition::CleanUp,
                "Clean Up",
            );
            self.draw_state_change_button(
                ui,
                self.is_enabled(&NetworkTransition::Activate),
                NetworkTransition::Activate,
                "Activate",
            );
            self.draw_state_change_button(
                ui,
                self.is_enabled(&NetworkTransition::Deactivate),
                NetworkTransition::Deactivate,
                "Deactivate",
            );
            self.draw_state_change_button(ui, self.is_enabled(&NetworkTransition::Deactivate), NetworkTransition::Arm, "Arm");
            self.draw_state_change_button(
                ui,
                self.is_enabled(&NetworkTransition::Deactivate),
                NetworkTransition::Disarm,
                "Disarm",
            );
            self.draw_state_change_button(
                ui,
                self.is_enabled(&NetworkTransition::Shutdown),
                NetworkTransition::Shutdown,
                "Shut Down",
            );
        });
    }

    fn update_data(&mut self, data: &Value) {
        match serde_json::from_value::<rctrl_rosbridge::rstate::srv::get_available_network_transitions::Response>(data.clone()) {
            Ok(values) => {
                self.available_transitions = values.available_transitions;
            }
            Err(e) => {
                log!(format!(
                    "NetworkStateTransitions::update_data() for node \"\\rstate\" failed: {}",
                    e
                ));
            }
        }
    }

    fn deregister(&self) {
        self.ws_lock.remove_gui_elem(self.op.clone(), self.topic.clone(), self.id.clone())
    }
}
