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
pub struct LifecycleManager {
    ws_lock: Rc<WsLock>,
    lifecycle_panels: Rc<Mutex<LifecyclePanels>>,
}

impl LifecycleManager {
    pub fn new(ws_lock: &Rc<WsLock>) -> Self {
        Self {
            ws_lock: Rc::clone(&ws_lock),
            lifecycle_panels: LifecyclePanels::new_shared(ws_lock),
        }
    }
}

impl epi::App for LifecycleManager {
    fn name(&self) -> &str {
        "lifecycle_manager"
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
        let Self { ws_lock, lifecycle_panels } = self;

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.horizontal(|ui| {
                let tooltip = "Refresh ROS2 node list";
                if ui.button("🔄").on_hover_text(tooltip).clicked() {
                    let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new("/rosapi/nodes");
                    self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
                }

                {
                    let lifecycle_panels = self.lifecycle_panels.lock().unwrap();
                    let refresh_msg = match lifecycle_panels.last_refresh {
                        Some(last_refresh) => format!("Refreshed {}ms ago", lifecycle_panels.performance.now() - last_refresh),
                        None => "Refreshed N/A ago".to_string(),
                    };

                    ui.with_layout(egui::Layout::left_to_right(), |ui| {
                        ui.label(refresh_msg);
                    });
                }
            });
            ui.separator();

            // Need to explicitly split up the borrowing here or else the borrow checker will complain about
            // borrowing an already mutable reference to self
            // REFERENCE: <https://doc.rust-lang.org/nomicon/borrow-splitting.html>
            {
                let lifecycle_panels = self.lifecycle_panels.lock().unwrap();
                egui::ScrollArea::vertical()
                    .auto_shrink([false; 2])
                    .show(ui, |ui| lifecycle_panels.draw(ui));
            }
        });
    }
}

impl App for LifecycleManager {
    fn refresh(&mut self) {
        // Call for refresh
        let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new("/rosapi/nodes");
        self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
    }
}

pub struct LifecyclePanels {
    ws_lock: Rc<WsLock>,
    window: Window,
    performance: Performance,
    last_refresh: Option<f64>,
    node_panels: Rc<Mutex<HashMap<String, Rc<Mutex<NodePanel>>>>>,
}

impl LifecyclePanels {
    pub fn new_shared(ws_lock: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let window = web_sys::window().expect("should have a window in this context");
        let performance = window.performance().expect("performance should be available");
        let lifecycle_panels = Self {
            ws_lock: Rc::clone(&ws_lock),
            window: window,
            performance: performance,
            last_refresh: None,
            node_panels: Rc::new(Mutex::new(HashMap::new())),
        };
        let op = ("service_response").to_owned();
        let topic = ("/rosapi/nodes").to_owned();
        let lifecycle_panels_lock = Rc::new(Mutex::new(lifecycle_panels));
        let lifecycle_panels_lock_c = Rc::clone(&lifecycle_panels_lock);
        ws_lock.add_gui_elem(op, topic, lifecycle_panels_lock_c);

        return lifecycle_panels_lock;
    }
}

impl GuiElem for LifecyclePanels {
    fn draw(&self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            for (key, value) in self.node_panels.lock().unwrap().iter() {
                ui.label(format!(""));
                value.lock().unwrap().draw(ui);

                // Break wrap if available space is smaller than the width of the next panel
                if ui.available_size_before_wrap().x <= 150.0 {
                    ui.end_row();
                }
            }
        });
    }

    fn update_data(&mut self, data: Value) {
        // Get comlete list of nodes
        let deserialized: rctrl_rosbridge::rosapi_msgs::srv::nodes::Response = serde_json::from_value(data).unwrap();

        // Retain all node_panels whos names exist in the respone, drop all others
        self.node_panels.lock().unwrap().retain(|k, _| deserialized.nodes.contains(k));

        // Create and insert new node_panels if they do node exist in the map
        for node in deserialized.nodes {
            self.node_panels
                .lock()
                .unwrap()
                .entry(node.clone())
                .or_insert_with(|| NodePanel::new_shared(node.clone(), &self.ws_lock));

            match node.as_str() {
                "/rosapi" => (),
                "/rosapi_params" => (),
                "/rosbridge_websocket" => (),
                _ => {
                    // Request state of node
                    let topic = node.clone() + "/get_state";
                    let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
                    self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

                    // Request available transitions of node
                    let topic = node.clone() + "/get_available_transitions";
                    let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
                    self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
                }
            }
        }

        // Update last refresh counter
        self.last_refresh = Some(self.performance.now());
    }
}

/// Wrapper structure containing all [`GuiElem`]'s needed for a single node.
struct NodePanel {
    ws_lock: Rc<WsLock>,
    pub node: String,
    state_display: Rc<Mutex<StateDisplay>>,
    state_transitions: Rc<Mutex<StateTransitions>>,
}

impl NodePanel {
    pub fn new_shared(node: String, ws_lock: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let state_display = StateDisplay::new_shared(node.clone(), ws_lock);
        let state_transitions = StateTransitions::new_shared(node.clone(), ws_lock);

        let node_panel = Self {
            ws_lock: Rc::clone(&ws_lock),
            node: node.clone(),
            state_display: state_display,
            state_transitions: state_transitions,
        };

        let op = ("publish").to_owned();
        let topic = node.clone() + "/transition_event";

        match node.as_str() {
            "/rosapi" => (),
            "/rosapi_params" => (),
            "/rosbridge_websocket" => (),
            _ => {
                let cmd = rctrl_rosbridge::protocol::Subscribe::new(&topic);
                ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
            }
        }

        let node_panel_lock = Rc::new(Mutex::new(node_panel));
        let node_panel_lock_c = Rc::clone(&node_panel_lock);
        ws_lock.add_gui_elem(op, topic, node_panel_lock_c);

        return node_panel_lock;
    }
}

impl GuiElem for NodePanel {
    fn draw(&self, ui: &mut egui::Ui) {
        ui.group(|ui| {
            ui.set_width(150.0);
            ui.vertical_centered_justified(|ui| {
                ui.label(format!("{:?}", self.node));
                self.state_display.lock().unwrap().draw(ui);
                self.state_transitions.lock().unwrap().draw(ui);
            });
        });
    }

    fn update_data(&mut self, data: Value) {
        match self.node.as_str() {
            "/rosapi" => (),
            "/rosapi_params" => (),
            "/rosbridge_websocket" => (),
            _ => {
                // Request state of node
                let topic = self.node.clone() + "/get_state";
                let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
                self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());

                // Request available transitions of node
                let topic = self.node.clone() + "/get_available_transitions";
                let cmd = rctrl_rosbridge::protocol::CallService::<u8>::new(&topic);
                self.ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
            }
        }
    }
}

/// Display the [`State`] of a node.
pub struct StateDisplay {
    ws_lock: Rc<WsLock>,
    state: State,
}

impl StateDisplay {
    pub fn new_shared(node: String, ws_lock: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let state_display = Self {
            ws_lock: Rc::clone(&ws_lock),
            state: State::Unknown,
        };

        let op = ("service_response").to_owned();
        let topic = node + "/get_state";
        let state_display_lock = Rc::new(Mutex::new(state_display));
        let state_display_lock_c = Rc::clone(&state_display_lock);
        ws_lock.add_gui_elem(op, topic, state_display_lock_c);

        return state_display_lock;
    }
}

impl GuiElem for StateDisplay {
    fn draw(&self, ui: &mut egui::Ui) {
        ui.label(format!("{:?}", self.state));
    }

    fn update_data(&mut self, data: Value) {
        match serde_json::from_value::<rctrl_rosbridge::lifecycle_msgs::srv::get_state::Response>(data) {
            Ok(values) => self.state = values.current_state,
            Err(e) => {
                self.state = State::Unknown;
                log!(format!("StateDisplay::update_data() failed: {}", e));
            }
        }
    }
}

/// Display the [`Transitions`] of a node.
pub struct StateTransitions {
    ws_lock: Rc<WsLock>,
    node: String,
    topic_change_state: String,
    available_transitions: Vec<TransitionDescription>,
}

impl StateTransitions {
    pub fn new_shared(node: String, ws_lock: &Rc<WsLock>) -> Rc<Mutex<Self>> {
        let topic_change_state = node.clone() + "/change_state";

        let state_transitions = Self {
            ws_lock: Rc::clone(&ws_lock),
            node: node.clone(),
            topic_change_state: topic_change_state,
            available_transitions: Vec::new(),
        };

        let op = ("service_response").to_owned();
        let topic = node + "/get_available_transitions";
        let state_transitions_lock = Rc::new(Mutex::new(state_transitions));
        let state_transitions_lock_c = Rc::clone(&state_transitions_lock);
        ws_lock.add_gui_elem(op, topic, state_transitions_lock_c);

        return state_transitions_lock;
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

impl GuiElem for StateTransitions {
    fn draw(&self, ui: &mut egui::Ui) {
        self.draw_state_change_button(ui, self.is_enabled(&Transition::Create), Transition::Create, "Create");
        self.draw_state_change_button(ui, self.is_enabled(&Transition::Configure), Transition::Configure, "Configure");
        self.draw_state_change_button(ui, self.is_enabled(&Transition::CleanUp), Transition::CleanUp, "Clean Up");
        self.draw_state_change_button(ui, self.is_enabled(&Transition::Activate), Transition::Activate, "Activate");
        self.draw_state_change_button(ui, self.is_enabled(&Transition::Deactivate), Transition::Deactivate, "Deactivate");
        self.draw_shutdown_button(ui);
        self.draw_state_change_button(ui, self.is_enabled(&Transition::Destroy), Transition::Destroy, "Destroy");
    }

    fn update_data(&mut self, data: Value) {
        match serde_json::from_value::<rctrl_rosbridge::lifecycle_msgs::srv::get_available_transitions::Response>(data) {
            Ok(values) => {
                self.available_transitions = values.available_transitions;
            }
            Err(e) => {
                log!(format!("StateTransitions::update_data() for node \"{}\" failed: {}", self.node, e));
            }
        }
    }
}
