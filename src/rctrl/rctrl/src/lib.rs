/*!
# rctrl
**rctrl** is a Rust application that targets WASM
Its goals are to:
- Establish a WebSocket connection to a rosbridge server
- Analyze the connected ROS2 network topology
- Provide a GUI to control the ROS2 network
- Provide visual feedback to the state of the ROS2 network

# Building rctrl
Currently only a testing build pipline exists.
The helper script `web_buil.sh` coordinated the build and optimization of the wasm package, while
`web_run.sh` starts a webserver to host the compiled application.
These scripts are based off the example found in the [eframe_template](https://github.com/emilk/eframe_template).
Included in the `.tasks.json` of the monorepo are the commands needed to build and serve the application.
*/

/// Application module.
mod app;

/// Gui module.
mod gui;

/// WsLock module.
mod ws_lock;
