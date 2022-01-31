use reqwasm::websocket::{futures::WebSocket, Message};
use futures::stream::SplitSink;
use futures::SinkExt;
use std::collections::VecDeque;
use std::sync::RwLock;
use std::rc::Rc;

pub async fn write_msg_queue(
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
