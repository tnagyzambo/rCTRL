#!/bin/bash
set -eu
echo "open http://localhost:8080"
(cd dist && basic-http-server --addr 127.0.0.1:8080 .)
