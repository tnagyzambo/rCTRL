use wasm_bindgen::prelude::*;
use futures::stream::SplitSink;
use futures::{SinkExt, StreamExt};
use gloo::timers::future::TimeoutFuture;
use gloo_console::log;
use reqwasm::websocket::{futures::WebSocket, Message, WebSocketError};
use std::collections::HashMap;
use std::rc::Rc;
use std::sync::RwLock;
use wasm_bindgen_futures::spawn_local;
use std::collections::VecDeque;

use serde::de::Deserialize;
use serde::de::value::MapDeserializer;

mod client;
mod gui;

use eframe::epi;

use gui::main::{GuiElem, GuiThing, GuiThot, RctrlGUI};
use rosbridge::protocol::{OpWrapper, PublishWrapper};

#[wasm_bindgen]
pub fn start() -> Result<(), eframe::wasm_bindgen::JsValue> {
    let ws = WebSocket::open("ws://127.0.0.1:9090").unwrap();
    let (mut ws_write, mut ws_read) = ws.split();

    // vars mutable from websocket
    let ws_read_lock = Rc::new(RwLock::new(HashMap::<String, Box<dyn GuiElem>>::new()));
    let ws_read_lock_c = Rc::clone(&ws_read_lock);

    // Websocket write buffer
    let ws_write_lock = Rc::new(RwLock::new(VecDeque::<String>::with_capacity(100)));
    let ws_write_lock_c = Rc::clone(&ws_write_lock);

    {
        let mut hash_map = ws_read_lock.write().unwrap();
        hash_map.insert(String::from("/rosout"), Box::new(GuiThing::new()));
        hash_map.insert(String::from("that"), Box::new(GuiThot::new()));
    }

    // Async read loop
    // spawn_local reaches out from wasm to the javascript glue code to execute a rust future on the current thread
    // Continually processes any messages recieved over the websocket
    // Does not block while waiting for messages
    spawn_local(async move {
        log!("starting...");
        while let Some(msg) = ws_read.next().await {
            read(msg, &ws_read_lock);
        }
        log!("WebSocket Closed");
    });

    // this is needed to subscribe to logging
    {
        let cmd = rosbridge::protocol::Subscribe::new("/rdata/transition_event");
        let mut test = ws_write_lock.write().unwrap();
        test.push_back(serde_json::to_string(&cmd).expect("Failed to serialise"));
    }

    {
        let cmd = rosbridge::protocol::Subscribe::new("/rosout");
        let mut test = ws_write_lock.write().unwrap();
        test.push_back(serde_json::to_string(&cmd).expect("Failed to serialise"));
    }

    // Async write loop
    // spawn_local reaches out from wasm to the javascript glue code to execute a rust future on the current thread
    // TimeoutFuture::new().await reaches out to javascript to run a timer on a new thread that will return a future on the current thread
    // Periodically writes all the messages in the write queue to the websocket
    spawn_local(async move {
        loop {
            ws_write_msg_queue(&mut ws_write, &ws_write_lock).await;
            TimeoutFuture::new(3_000).await;
            log!("hi!");
        }
    });

    let gui = RctrlGUI::new(ws_read_lock_c, ws_write_lock_c);
    eframe::start_web("rctrl_canvas", Box::new(gui))
}

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

async fn ws_write_msg_queue(
    ws_write: &mut SplitSink<WebSocket, Message>,
    ws_write_lock: &Rc<RwLock<VecDeque<String>>>,
) {
    let mut msg_queue = ws_write_lock.write().unwrap();
    match msg_queue.pop_front() {
        Some(msg) => {
            ws_write.send(Message::Text(msg)).await.unwrap();
        }
        None => (),
    };
}
