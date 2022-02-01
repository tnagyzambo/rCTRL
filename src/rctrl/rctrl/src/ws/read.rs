use gloo_console::log;
use reqwasm::websocket::{Message, WebSocketError};
use serde::de::{value::MapDeserializer, Deserialize};
use std::rc::Rc;
use std::sync::RwLock;

use rctrl_gui::GuiElems;
use rctrl_rosbridge::protocol::{OpWrapper, PublishWrapper, ServiceResponseWrapper};

struct ParseError<'a> {
    reason: &'a str,
    json: Option<String>,
}

enum ReadError<'a> {
    WsError(WebSocketError),
    ParseError(ParseError<'a>),
}

impl<'a> From<WebSocketError> for ReadError<'a> {
    fn from(e: WebSocketError) -> Self {
        ReadError::WsError(e)
    }
}

impl<'a> From<ParseError<'a>> for ReadError<'a> {
    fn from(e: ParseError<'a>) -> Self {
        ReadError::ParseError(e)
    }
}

pub fn read(
    msg: Result<Message, WebSocketError>,
    gui_elem_lock: &Rc<RwLock<GuiElems<String, String>>>,
) {
    match try_read(msg, gui_elem_lock) {
        Ok(()) => (),
        Err(e) => {
            //let mut hash_map = gui_elem_lock.write().unwrap();
            // do stuff with gui_elem_lock to display error
            log!("read_error");
        }
    }
}

fn try_read(
    msg: Result<Message, WebSocketError>,
    gui_elem_lock: &Rc<RwLock<GuiElems<String, String>>>,
) -> Result<(), ReadError> {
    // Unwrap the original WebSocket read to see if it was sucessful
    let msg_data = match msg {
        Ok(data) => data,
        Err(e) => return Err(ReadError::from(e)),
    };

    // Message is an enum that can either represent a String of a vector of Bytes
    // For now we only care about String messages
    // REFERENCE: <https://docs.rs/reqwasm/0.4.0/reqwasm/websocket/enum.Message.html>
    let msg_text = match msg_data {
        Message::Text(text) => text,
        Message::Bytes(_) => {
            return Err(ReadError::from(ParseError {
                reason: "byte stream received",
                json: None,
            }))
        }
    };

    // Deserialize the "op" field of the incomming json
    // Pass partially deserialized msg into dispatch_op() to handle incomming messsage
    let msg_op = match serde_json::from_str::<OpWrapper>(&msg_text) {
        Ok(op) => op,
        Err(_) => {
            return Err(ReadError::from(ParseError {
                reason: "failed to deserialize op",
                json: Some(msg_text),
            }))
        }
    };

    return match msg_op.op.as_ref() {
        //"fragment" => (),
        //"png" => (),
        //"set_level" => (),
        //"status" => (),
        //"auth" => (),
        //"advertise" => (),
        //"unadvertise" => (),
        "publish" => deserialize_publish(msg_text, msg_op, gui_elem_lock),
        //"subscribe" => (),
        //"unsubscribe" => (),
        //"call_service" => (),
        //"advertise_service" => (),
        //"unadvertise_service" => (),
        "service_response" => deserialize_service_response(msg_text, msg_op, gui_elem_lock),
        _ => {
            return Err(ReadError::from(ParseError {
                reason: "unrecognized op",
                json: Some(msg_text),
            }));
        }
    };
}

fn deserialize_publish(
    msg_text: String,
    op_wrapper: OpWrapper,
    gui_elem_lock: &Rc<RwLock<GuiElems<String, String>>>,
) -> Result<(), ReadError> {
    // Attempt to deserialize all fields excluding op into a PublishWrapper struct
    // The PublishWrapper struct defers typecasting the generic msg field, instead leaving it as a serde Value
    // Final typecasting can then be done when the topic is known
    // REFERENCE: <https://github.com/serde-rs/serde/issues/1739>
    let publish_wrapper =
        match PublishWrapper::deserialize(MapDeserializer::new(op_wrapper.other.into_iter())) {
            Ok(deserialized) => deserialized,
            Err(_) => {
                return Err(ReadError::from(ParseError {
                    reason: "failed to parse publish operation",
                    json: Some(msg_text),
                }));
            }
        };

    let hash_map = gui_elem_lock.read().unwrap();

    // Match GuiElem with the key pair <publish, topic>
    // Final typecasting of msg field is done within GuiElem.update()
    match hash_map.get(&String::from("publish"), &publish_wrapper.topic) {
        Some(gui_elem) => gui_elem.write().unwrap().update_data(publish_wrapper.msg),
        None => log!(format!(
            "GuiElem coresponding to keypair <publish, {}> does not exist.",
            &publish_wrapper.topic
        )),
    }

    Ok(())
}

fn deserialize_service_response(
    msg_text: String,
    op_wrapper: OpWrapper,
    gui_elem_lock: &Rc<RwLock<GuiElems<String, String>>>,
) -> Result<(), ReadError> {
    let service_response_wrapper = match ServiceResponseWrapper::deserialize(MapDeserializer::new(
        op_wrapper.other.into_iter(),
    )) {
        Ok(deserialized) => deserialized,
        Err(_) => {
            return Err(ReadError::from(ParseError {
                reason: "failed to parse service_response operation",
                json: Some(msg_text),
            }));
        }
    };

    let hash_map = gui_elem_lock.read().unwrap();

    // Match GuiElem with the key pair <publish, topic>
    // Final typecasting of msg field is done within GuiElem.update()
    match hash_map.get(
        &String::from("service_response"),
        &service_response_wrapper.service,
    ) {
        Some(gui_elem) => gui_elem
            .write()
            .unwrap()
            .update_data(service_response_wrapper.values),
        None => log!(format!(
            "GuiElem coresponding to keypair <service_response, {}> does not exist.",
            &service_response_wrapper.service
        )),
    }

    Ok(())
}
