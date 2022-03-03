#!/bin/bash
set -eu
echo "open http://localhost:8070"
(cd target/wasm32-unknown-unknown/doc/rctrl && basic-http-server --addr 127.0.0.1:8070 .)
