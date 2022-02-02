use crate::gui::Gui;
use crate::ws_lock::WsLock;
use console_error_panic_hook;
use futures::{SinkExt, StreamExt};
use gloo::timers::future::TimeoutFuture;
use reqwasm::websocket::Message;
use std::panic;
use std::rc::Rc;
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::spawn_local;

fn my_init_function() {
    panic::set_hook(Box::new(console_error_panic_hook::hook));

    // ...
}

#[wasm_bindgen]
pub fn main() -> Result<(), eframe::wasm_bindgen::JsValue> {
    my_init_function();

    let ws = reqwasm::websocket::futures::WebSocket::open("ws://127.0.0.1:9090").unwrap();
    let (mut ws_write, mut ws_read) = ws.split();

    let ws = Rc::new(WsLock::new());

    // Async read loop
    // spawn_local reaches out from wasm to the javascript glue code to execute a rust future on the current thread
    // Continually processes any messages recieved over the websocket
    // Does not block while waiting for messages
    let ws_c = Rc::clone(&ws);
    spawn_local(async move {
        while let Some(msg) = ws_read.next().await {
            ws_c.read(msg);
        }
    });

    // Async write loop
    // spawn_local reaches out from wasm to the javascript glue code to execute a rust future on the current thread
    // TimeoutFuture::new().await reaches out to javascript to run a timer on a new thread that will return a future on the current thread
    // Periodically writes all the messages in the write queue to the websocket
    let ws_c = Rc::clone(&ws);
    spawn_local(async move {
        loop {
            match ws_c.write_msg_que_pop() {
                Some(msg) => {
                    ws_write.send(Message::Text(msg)).await.unwrap();
                }
                None => (),
            };
            TimeoutFuture::new(500).await;
        }
    });

    // this is needed to subscribe to logging
    {
        let cmd = rctrl_rosbridge::protocol::Subscribe::new("/rosout");
        ws.add_ws_write(serde_json::to_string(&cmd).unwrap());
    }

    let ws_c = Rc::clone(&ws);
    let gui = Gui::new(ws_c);
    eframe::start_web("rctrl_canvas", Box::new(gui))
}
