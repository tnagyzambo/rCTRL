use reqwasm::websocket::{Message, WebSocketError};
use serde::de::{Deserialize, value::MapDeserializer};
use gloo_console::log;
use std::collections::HashMap;
use std::sync::RwLock;
use std::rc::Rc;

use rctrl_gui::main::GuiElem;
use rosbridge::protocol::{OpWrapper, PublishWrapper};

pub fn read(
    msg: Result<Message, WebSocketError>,
    ws_read_lock: &Rc<RwLock<HashMap<String, Box<dyn GuiElem>>>>,
) {
    match msg {
        Ok(msg_data) => {
            // Message is an enum that can either represent a String of a vector of Bytes
            // For now we only care about String messages
            // REFERENCE <https://github.com/serde-rs/serde/issues/1739>
            if let Message::Text(msg_text) = msg_data {
                // Deserialize the "op" field of the incomming json
                // Pass partially deserialized msg into dispatch_op() to handle incomming messsage
                match serde_json::from_str::<OpWrapper>(&msg_text) {
                    Ok(msg_op) => dispatch_op(msg_op, ws_read_lock),
                    Err(_) => log!("WebSocket read error: Error parsing received json"),
                }
            } else {
                log!("WebSocket read error: Byte stream received");
            }
        }
        Err(_) => log!("WebSocket error"),
    }
}

fn dispatch_op(msg: OpWrapper, ws_read_lock: &Rc<RwLock<HashMap<String, Box<dyn GuiElem>>>>) {
    match msg.op.as_ref() {
        "fragment" => (),
        "png" => (),
        "set_level" => (),
        "status" => (),
        "auth" => (),
        "advertise" => (),
        "unadvertise" => (),
        "publish" => dispatch_publish(msg, ws_read_lock),
        "subscribe" => (),
        "unsubscribe" => (),
        "call_service" => (),
        "advertise_service" => (),
        "unadvertise_service" => (),
        "service_response" => (),
        _ => log!("WebSocket read error: Unrecognized operation"),
    }
}

fn dispatch_publish(msg: OpWrapper, ws_read_lock: &Rc<RwLock<HashMap<String, Box<dyn GuiElem>>>>) {
    match PublishWrapper::deserialize(MapDeserializer::new(msg.other.into_iter())) {
        Ok(msg_publish) => dispatch_topic(msg_publish, ws_read_lock),
        Err(_) => log!("WebSocket read error: Error parsing publish operation"),
    }

    // match msg_publish_t.topic.as_ref() {
    //     "/rosout" => (), //let test: rosbridge::rosout::msg::Log = serde_json::from_value(msg_publish_t.msg),
    //     _ => log!("Unrecognized topic"),
    // }
}

fn dispatch_topic(
    msg: PublishWrapper,
    ws_read_lock: &Rc<RwLock<HashMap<String, Box<dyn GuiElem>>>>,
) {
    let mut hash_map = ws_read_lock.write().unwrap();

    match hash_map.get_mut(&msg.topic) {
        Some(gui_elem) => gui_elem.up(9.0),
        None => println!("GUI elem does not exist."),
    }
}
