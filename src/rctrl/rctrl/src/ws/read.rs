use reqwasm::websocket::{Message, WebSocketError};
use serde::de::{Deserialize, value::MapDeserializer};
use gloo_console::log;
use std::collections::HashMap;
use std::sync::RwLock;
use std::rc::Rc;

use rctrl_gui::{GuiElem, GuiElems};
use rctrl_rosbridge::protocol::{OpWrapper, PublishWrapper, ServiceResponseWrapper};

pub fn read(
    msg: Result<Message, WebSocketError>,
    ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>,
) {
    match msg {
        Ok(msg_data) => {
            // Message is an enum that can either represent a String of a vector of Bytes
            // For now we only care about String messages
            // REFERENCE <https://github.com/serde-rs/serde/issues/1739>
            if let Message::Text(msg_text) = msg_data {
                log!(&msg_text);
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

fn dispatch_op(msg: OpWrapper, ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>) {
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
        "service_response" => blah1(msg, ws_read_lock),
        _ => log!("WebSocket read error: Unrecognized operation"),
    }
}

fn dispatch_publish(op_wrapper: OpWrapper, ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>) {
    match PublishWrapper::deserialize(MapDeserializer::new(op_wrapper.other.into_iter())) {
        Ok(publish_wrapper) => dispatch_topic(publish_wrapper, ws_read_lock),
        Err(_) => log!("WebSocket read error: Error parsing publish operation"),
    }
}

fn dispatch_topic(
    publish_wrapper: PublishWrapper,
    ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>,
) {
    let mut hash_map = ws_read_lock.write().unwrap();

    match hash_map.get_mut(&String::from("publish"), &publish_wrapper.topic) {
        Some(gui_elem) => gui_elem.update_data(publish_wrapper.msg),
        None => println!("GUI elem does not exist."),
    }
}

fn blah1(op_wrapper: OpWrapper, ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>) {
    log!("hi from blah1");
    match ServiceResponseWrapper::deserialize(MapDeserializer::new(op_wrapper.other.into_iter())) {
        Ok(publish_wrapper) => blah2(publish_wrapper, ws_read_lock),
        Err(_) => log!("WebSocket read error: Error parsing publish operation"),
    }
}

fn blah2(
    publish_wrapper: ServiceResponseWrapper,
    ws_read_lock: &Rc<RwLock<GuiElems<String, String>>>,
) {
    let mut hash_map = ws_read_lock.write().unwrap();
    log!("hi from blah2");
    match hash_map.get_mut(&String::from("service_response"), &publish_wrapper.service) {
        Some(gui_elem) => gui_elem.update_data(publish_wrapper.values),
        None => println!("GUI elem does not exist."),
    }
}
