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

/// Main loop of the application
///
/// The `spawn_local(async move {})` must capture and own the `ws_read` and `ws_write` objects.
/// The locks used with in the async thread must be cloned in using a reference counted pointer.
/// Attempting to pass `&` references into the async threads will quickly result in lifetime issues as
/// a `'static` lifetime is required.
/// The [shared state design docs](https://tokio.rs/tokio/tutorial/shared-state) are a must read before working
/// on the async threads and the locks in this code.
///
/// Additional complications come from compiling to WASM. Currently WASM code is not allowed to be blocking.
/// All non-block behaviour such as the delay in the write loop is accomplished by reaching out to the JS
/// enviroment that the WASM application runs in. The native Rust code must not block.
#[wasm_bindgen]
pub fn main() -> Result<(), eframe::wasm_bindgen::JsValue> {
    // Set panic hook for addition debug information in web browser
    panic::set_hook(Box::new(console_error_panic_hook::hook));

    let ws = reqwasm::websocket::futures::WebSocket::open("ws://127.0.0.1:9090").unwrap();
    let (mut ws_write, mut ws_read) = ws.split();

    let ws_lock = Rc::new(WsLock::new());

    // Async read loop
    // spawn_local reaches out from wasm to the javascript glue code to execute a rust future on the current thread
    // Continually processes any messages recieved over the websocket
    // Does not block while waiting for messages
    let ws_lock_c = Rc::clone(&ws_lock);
    spawn_local(async move {
        while let Some(msg) = ws_read.next().await {
            ws_lock_c.read(msg);
        }
    });

    // Async write loop
    // spawn_local reaches out from wasm to the javascript glue code to execute a rust future on the current thread
    // TimeoutFuture::new().await reaches out to javascript to run a timer on a new thread that will return a future on the current thread
    // Periodically writes all the messages in the write queue to the websocket
    let ws_lock_c = Rc::clone(&ws_lock);
    spawn_local(async move {
        loop {
            match ws_lock_c.write_msg_que_pop() {
                Some(msg) => {
                    ws_write.send(Message::Text(msg)).await.unwrap();
                }
                None => (),
            };
            TimeoutFuture::new(50).await;
        }
    });

    // this is needed to subscribe to logging
    {
        let cmd = rctrl_rosbridge::protocol::Subscribe::new("/rosout");
        ws_lock.add_ws_write(serde_json::to_string(&cmd).unwrap());
    }

    let gui = Gui::new(&ws_lock);
    eframe::start_web("rctrl_canvas", Box::new(gui))
}
