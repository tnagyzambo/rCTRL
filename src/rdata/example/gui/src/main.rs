use futures::stream::SplitSink;
use futures::{SinkExt, StreamExt};
use gloo::timers::future::TimeoutFuture;
use gloo_console::log;
use reqwasm::websocket::{futures::WebSocket, Message, WebSocketError};
use std::collections::HashMap;
use std::rc::Rc;
use std::sync::RwLock;
use wasm_bindgen_futures::spawn_local;

use serde::de::Deserialize;
use serde::de::value::MapDeserializer;

mod client;
mod gui;

use gui::main::{GuiThing, GuiThot, MyTrait, RctrlGUI};

fn main() -> Result<(), eframe::wasm_bindgen::JsValue> {
    let ws = WebSocket::open("ws://127.0.0.1:9090").unwrap();
    let (mut ws_write, mut ws_read) = ws.split();

    // vars mutable from websocket
    let ws_read_lock = Rc::new(RwLock::new(HashMap::<String, Box<dyn MyTrait>>::new()));
    let ws_read_lock_c = Rc::clone(&ws_read_lock);

    // Websocket write buffer
    let ws_write_lock = Rc::new(RwLock::new(Vec::<String>::new()));
    let ws_write_lock_c = Rc::clone(&ws_write_lock);

    {
        let mut hash_map = ws_read_lock.write().unwrap();
        hash_map.insert(String::from("this"), Box::new(GuiThing::new()));
        hash_map.insert(String::from("that"), Box::new(GuiThot::new()));
    }

    // Async read loop
    // spawn_local reaches out from wasm to the javascript glue code to execute a rust future on the current thread
    // Continually processes any messages recieved over the websocket
    // Does not block while waiting for messages
    spawn_local(async move {
        log!("starting...");
        while let Some(msg) = ws_read.next().await {
            ws_read_msg(msg, &ws_read_lock);
        }
        log!("WebSocket Closed");
    });

    // this is needed to subscribe to logging
    {
        let cmd = rosbridge::protocol::Subscribe::new("/rdata/transition_event");
        let mut test = ws_write_lock.write().unwrap();
        test.push(serde_json::to_string(&cmd).expect("Failed to serialise"));
    }

    {
        let cmd = rosbridge::protocol::Subscribe::new("/rosout");
        let mut test = ws_write_lock.write().unwrap();
        test.push(serde_json::to_string(&cmd).expect("Failed to serialise"));
    }

    // Async write loop
    // spawn_local reaches out from wasm to the javascript glue code to execute a rust future on the current thread
    // TimeoutFuture::new().await reaches out to javascript to run a timer on a new thread that will return a future on the current thread
    // Periodically writes all the messages in the write queue to the websocket
    spawn_local(async move {
        loop {
            ws_write_msg_queue(&mut ws_write, &ws_write_lock).await;
            TimeoutFuture::new(1_000).await;
        }
    });

    let gui = RctrlGUI::new(ws_read_lock_c, ws_write_lock_c);
    eframe::start_web("rctrl_canvas", Box::new(gui))
}

fn ws_read_msg(
    msg: Result<Message, WebSocketError>,
    ws_read_lock: &Rc<RwLock<HashMap<String, Box<dyn MyTrait>>>>,
) {
    let mut hash_map = ws_read_lock.write().unwrap();
    match hash_map.get_mut(&String::from("this")) {
        Some(thing) => thing.up(9.0),
        None => println!("elem does not exist."),
    }





    // Message is an enum that can either represent a String of a vector of Bytes
    // For now we only care about String messages
    // REFERENCE <https://github.com/serde-rs/serde/issues/1739>
    if let Message::Text(msg_text) = msg.unwrap() {
        log!(&msg_text);
        let msg_t: rosbridge::protocol::Message = serde_json::from_str(&msg_text).unwrap();
        match msg_t.op.as_ref() {
            "publish" => {
                //log!(format!("{:?}", &msg_t.other));
                let msg_publish_t = rosbridge::protocol::MessagePublish::deserialize(MapDeserializer::new(msg_t.other.into_iter())).unwrap();
                match msg_publish_t.topic.as_ref() {
                    "/rosout" => {
                        //log!(format!("{:?}", &msg_publish_t.msg));
                        let test: rosbridge::rosout::msg::Log = serde_json::from_value(msg_publish_t.msg).unwrap();
                        //log!(format!("{:?}", test));
                    },
                    _ => log!("Unrecognized topic"),
                }
            },
            _ => log!("Unrecognized operation"),
        }
    }
}

async fn ws_write_msg_queue(
    ws_write: &mut SplitSink<WebSocket, Message>,
    ws_write_lock: &Rc<RwLock<Vec<String>>>,
) {
    let mut msg_queue = ws_write_lock.write().unwrap();
    match msg_queue.pop() {
        Some(msg) => {
            ws_write.send(Message::Text(msg)).await.unwrap();
        }
        None => (),
    };
}
