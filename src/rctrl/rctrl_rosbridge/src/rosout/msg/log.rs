use serde_derive::Deserialize;

#[derive(Deserialize, Debug)]
pub struct Log {
    pub stamp: Stamp,
    pub level: i64,
    pub name: String,
    pub msg: String,
    pub file: String,
    pub function: String,
    pub line: i64,
}

#[derive(Deserialize, Debug)]
pub struct Stamp {
    pub sec: i64,
    pub nanosec: i64,
}
