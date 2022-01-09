use reqwasm::websocket::{Message, futures::WebSocket};
use gloo_console::log;
use gloo::timers::future::TimeoutFuture;
use wasm_bindgen_futures::spawn_local;
use futures::{SinkExt, StreamExt};
use futures::stream::{SplitSink, SplitStream};
use futures_util::future::ready;
use eframe::epi::App;
use std::rc::Rc;
use std::sync::RwLock;
use std::{thread, time};

mod client;
mod gui;
mod rosbridge;

use gui::main::RctrlGUI;

fn main() -> Result<(), eframe::wasm_bindgen::JsValue> {
    let mut ws = WebSocket::open("ws://127.0.0.1:9090").unwrap();
    let (mut write, mut read) = ws.split();

    let lock = Rc::new(RwLock::new(5.0));
    let c_lock = Rc::clone(&lock);

    let write_lock = Rc::new(RwLock::new(Vec::<f32>::new()));
    let write_c_lock = Rc::clone(&write_lock);

    spawn_local(async move {
        loop {
            {
                let mut test = write_c_lock.write().unwrap();
                match test.pop() {
                    Some(k) => log!("{}", k),
                    None => (),
                };
                write.send(Message::Text(String::from("test"))).await.unwrap();
            }
            log!("hi!");
            TimeoutFuture::new(1_000).await;
        }
    });

    spawn_local(async move {
        {
            let mut w = lock.write().unwrap();
            *w = 6.0;
        }
        while let Some(msg) = read.next().await {
            log!(format!("1. {:?}", msg))
        }
        log!("WebSocket Closed");
    });
    
    let gui = RctrlGUI::new(c_lock, write_lock);
    
    eframe::start_web("the_canvas_id", Box::new(gui))
}

// // When compiling natively:
// #[cfg(not(target_arch = "wasm32"))]
// fn main() {
//     let app = hello_world::TemplateApp::default();
//     let native_options = eframe::NativeOptions::default();
//     eframe::run_native(Box::new(app), native_options);
// }

// use yew::prelude::*;


// use wasm_bindgen_futures::spawn_local;
// use futures::{SinkExt, StreamExt};


// enum Msg {
//     AddOne,
// }

// struct Model {
//     value: i64,
// }

// impl Component for Model {
//     type Message = Msg;
//     type Properties = ();

//     fn create(ctx: &Context<Self>) -> Self {
//         Self { value: 0 }
//     }

//     fn update(&mut self, _ctx: &Context<Self>, msg: Self::Message) -> bool {
//         match msg {
//             Msg::AddOne => {
//                 self.value += 1;
//                 true
//             }
//         }
//     }

//     fn view(&self, ctx: &Context<Self>) -> Html {
//         html! {
//             <div>
//                 <button onclick={ctx.link().callback(|_| Msg::AddOne)}>{ "+1" }</button>
//                 <p>{ self.value }</p>
//             </div>
//         }
//     }
// }

// fn main() {
//     let mut ws = WebSocket::open("ws://127.0.0.1:9090").unwrap();
//     let (mut write, mut read) = ws.split();
    
//     spawn_local(async move {
//         write.send(Message::Text(String::from("test"))).await.unwrap();
//     });
    
//     spawn_local(async move {
//         while let Some(msg) = read.next().await {
//             log!(format!("1. {:?}", msg))
//         }
//         log!("WebSocket Closed")
//     });

//     yew::start_app::<Model>();
// }

// fn main() {
//     ::std::env::set_var("RUST_LOG", "actix_web=info");
//     env_logger::init();

//     let sys = System::new("rosbridge_client");

//     yew::start_app::<Model>();

//     Arbiter::spawn(async {
//         let (response, framed) = Client::new()
//             .ws("ws://127.0.0.1:9090")
//             .connect()
//             .await
//             .map_err(|e| {
//                 println!("Error: {}", e);
//             })
//             .unwrap();

//         println!("{:?}", response);
//         let (sink, stream) = framed.split();
//         let addr = RosbridgeClient::create(|ctx| {
//             RosbridgeClient::add_stream(stream, ctx);
//             RosbridgeClient(SinkWrite::new(sink, ctx))
//         });

//         let transition = rosbridge::lifecycle_msgs::msg::Transition::new(1, "configure");
//         let msg = rosbridge::lifecycle_msgs::srv::ChangeStateRequest::new(&transition);

//         let cmd = rosbridge::protocol::CallService::new("rdata/change_state").with_args(&msg).cmd();
//         let cmd_json = serde_json::to_string(&cmd).expect("Failed to serialise");
//         println!("{}", cmd_json);
//         addr.do_send(ClientCommand(cmd_json));
//         return;
//     });
//     sys.run().unwrap();
// }

// struct RosbridgeClient(SinkWrite<Message, SplitSink<Framed<BoxedSocket, Codec>, Message>>);

// #[derive(Message)]
// #[rtype(result = "()")]
// struct ClientCommand(String);

// impl Actor for RosbridgeClient {
//     type Context = Context<Self>;

//     fn started(&mut self, ctx: &mut Context<Self>) {
//         // start heartbeats otherwise server will disconnect after 10 seconds
//         self.hb(ctx)
//     }

//     fn stopped(&mut self, _: &mut Context<Self>) {
//         println!("Disconnected");

//         // Stop application on disconnect
//         System::current().stop();
//     }
// }

// impl RosbridgeClient {
//     fn hb(&self, ctx: &mut Context<Self>) {
//         ctx.run_later(Duration::new(1, 0), |act, ctx| {
//             act.0.write(Message::Ping(Bytes::from_static(b"")));
//             act.hb(ctx);

//             // client should also check for a timeout here, similar to the
//             // server code
//         });
//     }
// }

// /// Handle stdin commands
// impl Handler<ClientCommand> for RosbridgeClient {
//     type Result = ();

//     fn handle(&mut self, msg: ClientCommand, _ctx: &mut Context<Self>) {
//         self.0.write(Message::Text(msg.0));
//     }
// }

// /// Handle server websocket messages
// impl StreamHandler<Result<Frame, WsProtocolError>> for RosbridgeClient {
//     fn handle(&mut self, msg: Result<Frame, WsProtocolError>, _: &mut Context<Self>) {
//         if let Ok(Frame::Text(txt)) = msg {
//             println!("Server: {:?}", txt)
//         }
//     }

//     fn started(&mut self, _ctx: &mut Context<Self>) {
//         println!("Connected");
//     }

//     fn finished(&mut self, ctx: &mut Context<Self>) {
//         println!("Server disconnected");
//         ctx.stop()
//     }
// }

// impl actix::io::WriteHandler<WsProtocolError> for RosbridgeClient {}
