use serde_derive::Deserialize;

#[derive(Deserialize, Debug)]
pub struct Log {
    stamp: Stamp,
    level: i64,
    name: String,
    msg: String,
    file: String,
    function: String,
    line: i64,
}

#[derive(Deserialize, Debug)]
pub struct Stamp {
    sec: i64,
    nanosec: i64,
}
