/// REFERENCE: http://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Log.html
use serde_derive::Deserialize;
use std::convert::TryFrom;

#[derive(Deserialize, Debug, PartialEq)]
pub struct Log {
    pub stamp: Stamp,
    pub level: Level,
    pub name: String,
    pub msg: String,
    pub file: String,
    pub function: String,
    pub line: i64,
}

#[derive(Deserialize, Debug, PartialEq)]
pub struct Stamp {
    pub sec: i64,
    pub nanosec: i64,
}

#[repr(u8)]
#[derive(Deserialize, PartialEq, Debug)]
#[serde(try_from = "u8")]
pub enum Level {
    Unset = 0,
    Debug = 10,
    Info = 20,
    Warn = 30,
    Error = 40,
    Fatal = 50,
}

impl TryFrom<u8> for Level {
    type Error = InvalidLevelError;

    fn try_from(level: u8) -> Result<Self, <Level as TryFrom<u8>>::Error> {
        if level == 0 || level == 10 || level == 20 || level == 30 || level == 40 || level == 50 {
            Ok(unsafe { std::mem::transmute(level) })
        } else {
            Err(InvalidLevelError)
        }
    }
}

#[derive(Debug, Clone)]
pub struct InvalidLevelError;

impl std::fmt::Display for InvalidLevelError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "out of bounds logging severity does not yeild a valid level")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn log() {
        let msg: Log = serde_json::from_str(
            r#"{"stamp":{"sec":1234,"nanosec":1234},"level":10,"name":"node","msg":"msg","file":"file","function":"function","line":1}"#,
        )
        .unwrap();

        let log = Log {
            stamp: Stamp { sec: 1234, nanosec: 1234 },
            level: Level::Debug,
            name: "node".to_string(),
            msg: "msg".to_string(),
            file: "file".to_string(),
            function: "function".to_string(),
            line: 1,
        };

        assert_eq!(msg, log);
    }

    #[test]
    #[should_panic]
    fn level_out_of_bounds() {
        let _msg: Log = serde_json::from_str(
            r#"{"stamp":{"sec":1234,"nanosec":1234},"level":11,"name":"node","msg":"msg","file":"file","function":"function","line":1}"#,
        )
        .unwrap();
    }
}
