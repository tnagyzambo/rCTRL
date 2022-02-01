use futures::StreamExt;
use gloo::timers::future::TimeoutFuture;
use gloo_console::log;
use reqwasm::websocket::futures::WebSocket;
use std::collections::VecDeque;
use std::rc::Rc;
use std::sync::RwLock;
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::spawn_local;

use rctrl_gui::{lc::StateDisplay, Gui, GuiElems};

mod ws;

#[wasm_bindgen]
pub fn main() -> Result<(), eframe::wasm_bindgen::JsValue> {
    let ws = WebSocket::open("ws://127.0.0.1:9090").unwrap();
    let (mut ws_write, mut ws_read) = ws.split();

    // vars mutable from websocket
    let ws_read_lock = Rc::new(RwLock::new(GuiElems::<String, String>::new()));
    let ws_read_lock_c = Rc::clone(&ws_read_lock);

    // Websocket write buffer
    let ws_write_lock = Rc::new(RwLock::new(VecDeque::<String>::with_capacity(100)));
    let ws_write_lock_c = Rc::clone(&ws_write_lock);

    {
        let mut hash_map = ws_read_lock.write().unwrap();
        hash_map.insert(
            String::from("service_response"),
            String::from("/rdata/get_state"),
            Box::new(StateDisplay::new()),
        );
    }

    // Async read loop
    // spawn_local reaches out from wasm to the javascript glue code to execute a rust future on the current thread
    // Continually processes any messages recieved over the websocket
    // Does not block while waiting for messages
    spawn_local(async move {
        log!("starting...");
        while let Some(msg) = ws_read.next().await {
            ws::read(msg, &ws_read_lock);
        }
        log!("WebSocket Closed");
    });

    // this is needed to subscribe to logging
    {
        let cmd = rctrl_rosbridge::protocol::Subscribe::new("/rdata/transition_event");
        let mut test = ws_write_lock.write().unwrap();
        test.push_back(serde_json::to_string(&cmd).expect("Failed to serialise"));
    }

    {
        let cmd = rctrl_rosbridge::protocol::Subscribe::new("/rosout");
        let mut test = ws_write_lock.write().unwrap();
        test.push_back(serde_json::to_string(&cmd).expect("Failed to serialise"));
    }

    // Async write loop
    // spawn_local reaches out from wasm to the javascript glue code to execute a rust future on the current thread
    // TimeoutFuture::new().await reaches out to javascript to run a timer on a new thread that will return a future on the current thread
    // Periodically writes all the messages in the write queue to the websocket
    spawn_local(async move {
        loop {
            ws::write_msg_queue(&mut ws_write, &ws_write_lock).await;
            TimeoutFuture::new(500).await;
        }
    });

    let gui = Gui::new(ws_read_lock_c, ws_write_lock_c);
    eframe::start_web("rctrl_canvas", Box::new(gui))
}
