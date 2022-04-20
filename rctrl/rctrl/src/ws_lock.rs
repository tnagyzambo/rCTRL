use crate::gui::{GuiElem, GuiElems};
use gloo_console::log;
use rctrl_rosbridge::protocol::{OpWrapper, PublishWrapper, ServiceResponseWrapper};
use reqwasm::websocket::{Message, WebSocketError};
use serde::de::{value::MapDeserializer, Deserialize};
use std::collections::VecDeque;
use std::fmt;
use std::rc::Rc;
use std::sync::Mutex;

/// Share-able structure for message passing.
///
/// All Mutex acessing is done through methods that are implemented by [`WsLock`].
/// This ensured that not deadlock conditions can be created between the acess in the
/// WebSocket read/write loops and the [`Gui`](crate::gui::Gui) draw loop.
pub struct WsLock {
    /// Lock of [`GuiElems`](crate::gui::gui_elem::GuiElems) map that will be updated when an appropriate `WebSocket` read has been made.
    gui_elems_lock: Mutex<GuiElems<String, String, u32>>,
    /// Lock of messages waiting to be writen to the WebSocket.
    ws_write_lock: Mutex<VecDeque<String>>,
}

impl Default for WsLock {
    fn default() -> Self {
        Self {
            gui_elems_lock: Mutex::default(),
            ws_write_lock: Mutex::default(),
        }
    }
}

impl WsLock {
    pub fn new() -> Self {
        Self {
            gui_elems_lock: Mutex::new(GuiElems::<String, String, u32>::new()),
            ws_write_lock: Mutex::new(VecDeque::<String>::with_capacity(100)),
        }
    }

    /// Only acessed from the async read loop in [`main()`](crate::app::main).
    pub fn read(&self, msg: Result<Message, WebSocketError>) {
        match self.try_read(msg) {
            Ok(()) => (),
            Err(e) => {
                //let mut hash_map = gui_elem_lock.write().unwrap();
                // do stuff with gui_elem_lock to display error
                log!(format!("Read Error: {}", e));
            }
        }
    }

    /// Only acessed from the async read loop in [`main()`](crate::app::main).
    pub fn write_msg_que_pop(&self) -> Option<String> {
        let mut msg_queue = self.ws_write_lock.lock().unwrap();
        let msg = msg_queue.pop_front();

        return msg;
    }

    /// Given a key pair, add an object that implements the [`GuiElem`](crate::gui::gui_elem::GuiElem) to the HashMap [`GuiElems`](crate::gui::gui_elem::GuiElems).
    pub fn add_gui_elem(&self, op: String, topic: String, id: u32, gui_elem: Rc<Mutex<dyn GuiElem>>) {
        let mut gui_elems = self.gui_elems_lock.lock().unwrap();
        gui_elems.insert(op, topic, id, gui_elem);
    }

    /// Given a key pair, remove the [`GuiElem`](crate::gui::gui_elem::GuiElem) from the HashMap [`GuiElems`](crate::gui::gui_elem::GuiElems).
    pub fn remove_gui_elem(&self, op: String, topic: String, id: u32) {
        let mut gui_elems = self.gui_elems_lock.lock().unwrap();
        gui_elems.remove(op, topic, id);
    }

    /// Add a message to the `WebSocket` write queue.
    pub fn add_ws_write(&self, msg: String) {
        let mut msg_queue = self.ws_write_lock.lock().unwrap();
        msg_queue.push_back(msg);
    }

    fn try_read(&self, msg: Result<Message, WebSocketError>) -> Result<(), ReadError> {
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
            "publish" => self.deserialize_publish(msg_text, msg_op),
            //"subscribe" => (),
            //"unsubscribe" => (),
            //"call_service" => (),
            //"advertise_service" => (),
            //"unadvertise_service" => (),
            "service_response" => self.deserialize_service_response(msg_text, msg_op),
            _ => {
                return Err(ReadError::from(ParseError {
                    reason: "unrecognized op",
                    json: Some(msg_text),
                }));
            }
        };
    }

    fn deserialize_publish(&self, msg_text: String, op_wrapper: OpWrapper) -> Result<(), ReadError> {
        // Attempt to deserialize all fields excluding op into a PublishWrapper struct
        // The PublishWrapper struct defers typecasting the generic msg field, instead leaving it as a serde Value
        // Final typecasting can then be done when the topic is known
        // REFERENCE: <https://github.com/serde-rs/serde/issues/1739>
        let publish_wrapper = match PublishWrapper::deserialize(MapDeserializer::new(op_wrapper.other.into_iter())) {
            Ok(deserialized) => deserialized,
            Err(_) => {
                return Err(ReadError::from(ParseError {
                    reason: "failed to parse publish operation",
                    json: Some(msg_text),
                }));
            }
        };

        // Get vector of key values present in inner map of <id, GuiElem> from key pair <node, topic>
        let gui_elems = self.gui_elems_lock.lock().unwrap();
        let gui_elems_ids = match gui_elems.get_ids(&String::from("publish"), &publish_wrapper.topic) {
            Some(ids) => ids.clone(),
            None => return Ok(()),
        };
        drop(gui_elems);

        // Iterate over id's to updat all GuiElems
        // You cannot directly take an iterator of the elements with in the inner map as that would lead to some weird self modifying behaviour
        // Iterating over a vector of copied id's allows the program to handle the case where the previous iteration of the loop deleted the GuiElem
        // that would have been referneced in the current interation
        for id in gui_elems_ids {
            let gui_elems = self.gui_elems_lock.lock().unwrap();

            match gui_elems.get(&String::from("publish"), &publish_wrapper.topic, &id) {
                Some(gui_elem) => {
                    // Must drop the lock before calling update_data() since update_data() might need to aquire the lock
                    drop(gui_elems);
                    gui_elem.lock().unwrap().update_data(&publish_wrapper.msg);
                }
                None => log!(format!(
                    "GuiElem coresponding to keypair <publish, {}, {}> does not exist.",
                    &publish_wrapper.topic, &id
                )),
            }
        }

        Ok(())
    }

    fn deserialize_service_response(&self, msg_text: String, op_wrapper: OpWrapper) -> Result<(), ReadError> {
        // Final typecasting of msg field is done within GuiElem.update()
        let service_response_wrapper = match ServiceResponseWrapper::deserialize(MapDeserializer::new(op_wrapper.other.into_iter())) {
            Ok(deserialized) => deserialized,
            Err(_) => {
                return Err(ReadError::from(ParseError {
                    reason: "failed to parse service_response operation",
                    json: Some(msg_text),
                }));
            }
        };

        // Get vector of key values present in inner map of <id, GuiElem> from key pair <node, topic>
        let gui_elems = self.gui_elems_lock.lock().unwrap();
        let gui_elems_ids = match gui_elems.get_ids(&String::from("service_response"), &service_response_wrapper.service) {
            Some(ids) => ids.clone(),
            None => return Ok(()),
        };
        drop(gui_elems);

        // Iterate over id's to updat all GuiElems
        // You cannot directly take an iterator of the elements with in the inner map as that would lead to some weird self modifying behaviour
        // Iterating over a vector of copied id's allows the program to handle the case where the previous iteration of the loop deleted the GuiElem
        // that would have been referneced in the current interation
        for id in gui_elems_ids {
            let gui_elems = self.gui_elems_lock.lock().unwrap();

            match gui_elems.get(&String::from("service_response"), &service_response_wrapper.service, &id) {
                Some(gui_elem) => {
                    // Must drop the lock before calling update_data() since update_data() might need to aquire the lock
                    drop(gui_elems);
                    gui_elem.lock().unwrap().update_data(&service_response_wrapper.values);
                }
                None => log!(format!(
                    "GuiElem coresponding to keypair <service_response, {}, {}> does not exist.",
                    &service_response_wrapper.service, &id
                )),
            }
        }

        Ok(())
    }
}

/// Main error structure.
enum ReadError<'a> {
    WsError(WebSocketError),
    ParseError(ParseError<'a>),
}

impl<'a> fmt::Display for ReadError<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "WebSocket read error")
    }
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

/// Parse errors durring `WebSocket` read.
struct ParseError<'a> {
    reason: &'a str,
    json: Option<String>,
}
