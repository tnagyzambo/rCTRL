//! Rosbridge protocol implementation.
//! REFERENCE: <https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md>

use serde_derive::{Deserialize, Serialize};
use std::collections::BTreeMap;
use serde_json::Value;
use std::convert::AsRef;
use strum_macros::AsRefStr;

// Struct to hold incoming rosbridge message
// Further deserialization can be done based on "op" field
// As the rosbridge protocol has a flat heirachy, most of the methods that are easily found for partial deserialization do not work
// REFERENCE: <https://github.com/serde-rs/serde/issues/941>
#[derive(Deserialize, Debug)]
pub struct Message {
    pub op: String,
    #[serde(flatten)]
    pub other: BTreeMap<String, Value>,
}

#[derive(Deserialize, Debug)]
pub struct MessagePublish {
    pub topic: String,
    pub msg: Value,
}

// Enum to provide distinction between owned and referenced pointers
// A pointer to `T` which may or may not own the data. When deserializing we always want to produce owned data.
// Serde untagged makes this enum transparent and hides Ref and Owned from the serialised output
#[derive(Serialize, Debug)]
#[serde(untagged)]
pub enum Ptr<'a, T: 'a + ?Sized> {
    #[allow(dead_code)]
    Ref(&'a T),
    Owned(Box<T>),
}

impl<'de, 'a, T: 'a + ?Sized> serde::Deserialize<'de> for Ptr<'a, T>
where
    Box<T>: serde::Deserialize<'de>,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        serde::Deserialize::deserialize(deserializer).map(Ptr::Owned)
    }
}

/// 3.2 Status Messages
/// Enum to define status levels
#[derive(AsRefStr)]
pub enum Level {
    #[strum(serialize = "info")]
    Info,
    #[strum(serialize = "warning")]
    Warning,
    #[strum(serialize = "error")]
    Error,
    #[strum(serialize = "none")]
    None,
}

/// 3.2.1 Set Status Level
#[derive(Serialize)]
pub struct SetLevel<'a> {
    op: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    id: Option<&'a str>,
    level: &'a str,
}

impl<'a> SetLevel<'a> {
    pub fn new(level: &'a Level) -> SetLevel<'a> {
        SetLevel {
            op: "set_level",
            id: None,
            level: level.as_ref(),
        }
    }

    pub fn with_id(&mut self, id: &'a str) -> &mut Self {
        self.id = Some(&id);
        self
    }

    pub fn cmd(&self) -> SetLevel<'a> {
        SetLevel {
            op: self.op,
            id: self.id,
            level: self.level,
        }
    }
}

/// 3.2.2 Status Message
#[derive(Deserialize)]
pub struct Status<'a> {
    op: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    id: Option<&'a str>,
    level: &'a str,
    msg: &'a str,
}

/// 3.3.2 Authenticate
#[derive(Serialize)]
pub struct Authenticate<'a> {
    op: &'a str,
    mac: &'a str,
    client: &'a str,
    dest: &'a str,
    rand: &'a str,
    t: i64,
    level: &'a str,
    end: i64,
}

impl<'a> Authenticate<'a> {
    pub fn new(
        mac: &'a str,
        client: &'a str,
        dest: &'a str,
        rand: &'a str,
        t: i64,
        level: &'a Level,
        end: i64,
    ) -> Authenticate<'a> {
        Authenticate {
            op: "auth",
            mac: mac,
            client: client,
            dest: dest,
            rand: rand,
            t: t,
            level: level.as_ref(),
            end: end,
        }
    }
}

/// 3.4.1 Advertise
#[derive(Serialize)]
pub struct Advertise<'a> {
    op: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    id: Option<&'a str>,
    topic: &'a str,
    r#type: &'a str,
}

impl<'a> Advertise<'a> {
    pub fn new(topic: &'a str, r#type: &'a str) -> Advertise<'a> {
        Advertise {
            op: "advertise",
            id: None,
            topic: topic,
            r#type: r#type,
        }
    }

    pub fn with_id(&mut self, id: &'a str) -> &mut Self {
        self.id = Some(&id);
        self
    }

    pub fn cmd(&self) -> Advertise<'a> {
        Advertise {
            op: self.op,
            id: self.id,
            topic: self.topic,
            r#type: self.r#type,
        }
    }
}

/// 3.4.2 Unadvertise
#[derive(Serialize)]
pub struct Unadvertise<'a> {
    op: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    id: Option<&'a str>,
    topic: &'a str,
}

impl<'a> Unadvertise<'a> {
    pub fn new(topic: &'a str) -> Unadvertise<'a> {
        Unadvertise {
            op: "unadvertise",
            id: None,
            topic: topic,
        }
    }

    pub fn with_id(&mut self, id: &'a str) -> &mut Self {
        self.id = Some(&id);
        self
    }

    pub fn cmd(&self) -> Unadvertise<'a> {
        Unadvertise {
            op: self.op,
            id: self.id,
            topic: self.topic,
        }
    }
}

/// 3.4.3 Publish
#[derive(Serialize)]
pub struct Publish<'a, T> {
    op: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    id: Option<&'a str>,
    topic: &'a str,
    msg: &'a T,
}

impl<'a, T> Publish<'a, T> {
    pub fn new(topic: &'a str, msg: &'a T) -> Publish<'a, T> {
        Publish {
            op: "publish",
            id: None,
            topic: topic,
            msg: msg,
        }
    }

    pub fn with_id(&mut self, id: &'a str) -> &mut Self {
        self.id = Some(&id);
        self
    }

    pub fn cmd(&self) -> Publish<'a, T> {
        Publish {
            op: self.op,
            id: self.id,
            topic: self.topic,
            msg: self.msg,
        }
    }
}

/// 3.4.4 Subscribe
#[derive(AsRefStr)]
pub enum Compression {
    #[strum(serialize = "none")]
    None,
    #[strum(serialize = "png")]
    Png,
}

#[derive(Serialize)]
pub struct Subscribe<'a> {
    op: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    id: Option<&'a str>,
    topic: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    r#type: Option<&'a str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    throttle_rate: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    queue_length: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    fragment_size: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    compression: Option<&'a str>,
}

impl<'a> Subscribe<'a> {
    pub fn new(topic: &'a str) -> Subscribe<'a> {
        Subscribe {
            op: "subscribe",
            id: None,
            topic: topic,
            r#type: None,
            throttle_rate: None,
            queue_length: None,
            fragment_size: None,
            compression: None,
        }
    }

    pub fn with_id(&mut self, id: &'a str) -> &mut Self {
        self.id = Some(&id);
        self
    }

    pub fn with_type(&mut self, r#type: &'a str) -> &mut Self {
        self.r#type = Some(&r#type);
        self
    }

    pub fn with_throttle_rate(&mut self, throttle_rate: i64) -> &mut Self {
        self.throttle_rate = Some(throttle_rate);
        self
    }

    pub fn with_queue_length(&mut self, queue_length: i64) -> &mut Self {
        self.queue_length = Some(queue_length);
        self
    }

    pub fn with_fragment_size(&mut self, fragment_size: i64) -> &mut Self {
        self.fragment_size = Some(fragment_size);
        self
    }

    pub fn with_compression(&mut self, compression: &'a Compression) -> &mut Self {
        self.compression = Some(compression.as_ref());
        self
    }

    pub fn cmd(&self) -> Subscribe<'a> {
        Subscribe {
            op: self.op,
            id: self.id,
            topic: self.topic,
            r#type: self.r#type,
            throttle_rate: self.throttle_rate,
            queue_length: self.queue_length,
            fragment_size: self.fragment_size,
            compression: self.compression,
        }
    }
}

/// 3.4.5 Unsubscribe
#[derive(Serialize)]
pub struct Unsubscribe<'a> {
    op: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    id: Option<&'a str>,
    topic: &'a str,
}

impl<'a> Unsubscribe<'a> {
    pub fn new(topic: &'a str) -> Unsubscribe<'a> {
        Unsubscribe {
            op: "unsubscribe",
            id: None,
            topic: topic,
        }
    }

    pub fn with_id(&mut self, id: &'a str) -> &mut Self {
        self.id = Some(&id);
        self
    }

    pub fn cmd(&self) -> Unsubscribe<'a> {
        Unsubscribe {
            op: self.op,
            id: self.id,
            topic: self.topic,
        }
    }
}

/// 3.4.6 Call Service
// The rosbridge command call_service is bidirectional, args must therefor support serialization and deserialization
// This requires that we make the distinction between owned and reference memory as during serialization we are referencing a generic type T that can be owned outside of the CallService struct
// compared to deserialization where the destination CallService struct becomes the owner of the newly created generic type T
// Serde requires hand written bounds in order to make sense of this call
// REFERENCE: <https://serde.rs/attr-bound.html>
//
// See Ptr enum for more details
#[derive(Serialize, Deserialize)]
pub struct CallService<'a, T> {
    op: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    id: Option<&'a str>,
    service: &'a str,
    #[serde(
        skip_serializing_if = "Option::is_none",
        bound(
            serialize = "Option<Ptr<'a, T>>: serde::Serialize",
            deserialize = "Option<Ptr<'a, T>>: serde::Deserialize<'de>"
        )
    )]
    args: Option<Ptr<'a, T>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    fragment_size: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    compression: Option<&'a str>,
}

impl<'a, T> CallService<'a, T> {
    pub fn new(service: &'a str) -> CallService<'a, T> {
        CallService {
            op: "call_service",
            id: None,
            service: service,
            args: None,
            fragment_size: None,
            compression: None,
        }
    }

    pub fn with_id(&mut self, id: &'a str) -> &mut Self {
        self.id = Some(&id);
        self
    }

    pub fn with_args(&mut self, args: &'a T) -> &mut Self {
        self.args = Some(Ptr::Ref(args));
        self
    }

    pub fn with_fragment_size(&mut self, fragment_size: i64) -> &mut Self {
        self.fragment_size = Some(fragment_size);
        self
    }

    pub fn with_compression(&mut self, compression: &'a Compression) -> &mut Self {
        self.compression = Some(compression.as_ref());
        self
    }

    pub fn cmd(&mut self) -> CallService<'a, T> {
        CallService {
            op: self.op,
            id: self.id,
            service: self.service,
            args: self.args.take(),
            fragment_size: self.fragment_size,
            compression: self.compression,
        }
    }
}

/// 3.4.7 Advertise Service
#[derive(Serialize)]
pub struct AdvertiseService<'a> {
    op: &'a str,
    r#type: &'a str,
    service: &'a str,
}

impl<'a> AdvertiseService<'a> {
    pub fn new(r#type: &'a str, service: &'a str) -> AdvertiseService<'a> {
        AdvertiseService {
            op: "advertise_service",
            r#type: r#type,
            service: service,
        }
    }
}

/// 3.4.8 Unadvertise Service
#[derive(Serialize)]
pub struct UnadvertiseService<'a> {
    op: &'a str,
    service: &'a str,
}

impl<'a> UnadvertiseService<'a> {
    pub fn new(service: &'a str) -> UnadvertiseService<'a> {
        UnadvertiseService {
            op: "unadvertise_service",
            service: service,
        }
    }
}

/// 3.4.9 Service Response
#[derive(Serialize, Deserialize)]
pub struct ServiceResponse<'a, T> {
    op: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    id: Option<&'a str>,
    service: &'a str,
    #[serde(
        skip_serializing_if = "Option::is_none",
        bound(
            serialize = "Option<Ptr<'a, T>>: serde::Serialize",
            deserialize = "Option<Ptr<'a, T>>: serde::Deserialize<'de>"
        )
    )]
    values: Option<Ptr<'a, T>>,
    result: bool,
}

impl<'a, T> ServiceResponse<'a, T> {
    pub fn new(service: &'a str, result: bool) -> ServiceResponse<'a, T> {
        ServiceResponse {
            op: "service_response",
            id: None,
            service: service,
            values: None,
            result: result,
        }
    }

    pub fn with_id(&mut self, id: &'a str) -> &mut Self {
        self.id = Some(&id);
        self
    }

    pub fn with_values(&mut self, values: &'a T) -> &mut Self {
        self.values = Some(Ptr::Ref(values));
        self
    }

    pub fn cmd(&mut self) -> ServiceResponse<'a, T> {
        ServiceResponse {
            op: self.op,
            id: self.id,
            service: self.service,
            values: self.values.take(),
            result: self.result,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Serialize, Deserialize)]
    struct TestStruct<'a> {
        string: &'a str,
        int: i64,
    }

    impl<'a> TestStruct<'a> {
        pub fn new() -> TestStruct<'a> {
            TestStruct {
                string: "test string",
                int: 5,
            }
        }
    }

    #[test]
    fn level() {
        // Test casting level to str
        let cmd = SetLevel::new(&Level::Info);
        assert_eq!(cmd.level, "info");
        let cmd = SetLevel::new(&Level::Warning);
        assert_eq!(cmd.level, "warning");
        let cmd = SetLevel::new(&Level::Error);
        assert_eq!(cmd.level, "error");
        let cmd = SetLevel::new(&Level::None);
        assert_eq!(cmd.level, "none");
    }

    #[test]
    fn set_level() {
        // Test no optional arguments
        let cmd = SetLevel::new(&Level::Info);
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(cmd_json, r#"{"op":"set_level","level":"info"}"#);

        // Test all arguments
        let cmd = SetLevel::new(&Level::Info).with_id("test").cmd();
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(cmd_json, r#"{"op":"set_level","id":"test","level":"info"}"#);
    }

    #[test]
    fn status() {
        let _status: Status =
            serde_json::from_str(r#"{"op":"status","id":"test","level":"info","msg":"test"}"#)
                .expect("Deserialization failed");
    }

    #[test]
    fn authenticate() {
        let cmd = Authenticate::new(
            "00:00:5e:00:53:af",
            "127.0.0.1",
            "127.0.0.1",
            "rand",
            1,
            &Level::Info,
            1,
        );

        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(
            cmd_json,
            r#"{"op":"auth","mac":"00:00:5e:00:53:af","client":"127.0.0.1","dest":"127.0.0.1","rand":"rand","t":1,"level":"info","end":1}"#
        );
    }

    #[test]
    fn advertise() {
        // Test no optional arguments
        let cmd = Advertise::new("/test/topic", "std_msgs/Example");
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(
            cmd_json,
            r#"{"op":"advertise","topic":"/test/topic","type":"std_msgs/Example"}"#
        );

        // Test all arguments
        let cmd = Advertise::new("/test/topic", "std_msgs/Example")
            .with_id("test")
            .cmd();
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(
            cmd_json,
            r#"{"op":"advertise","id":"test","topic":"/test/topic","type":"std_msgs/Example"}"#
        );
    }

    #[test]
    fn unadvertise() {
        // Test no optional arguments
        let cmd = Unadvertise::new("/test/topic");
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(cmd_json, r#"{"op":"unadvertise","topic":"/test/topic"}"#);

        // Test all arguments
        let cmd = Unadvertise::new("/test/topic").with_id("test").cmd();
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(
            cmd_json,
            r#"{"op":"unadvertise","id":"test","topic":"/test/topic"}"#
        );
    }

    #[test]
    fn publish() {
        let test_struct = TestStruct::new();

        // Test no optional arguments
        let cmd = Publish::new("/test/topic", &test_struct);
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(
            cmd_json,
            r#"{"op":"publish","topic":"/test/topic","msg":{"string":"test string","int":5}}"#
        );

        // Test all arguments
        let cmd = Publish::new("/test/topic", &test_struct)
            .with_id("test")
            .cmd();
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(
            cmd_json,
            r#"{"op":"publish","id":"test","topic":"/test/topic","msg":{"string":"test string","int":5}}"#
        );
    }

    #[test]
    fn compression() {
        // Test casting compression to str
        let cmd = Subscribe::new("/test/topic")
            .with_compression(&Compression::None)
            .cmd();
        assert_eq!(cmd.compression, Some("none"));
        let cmd = Subscribe::new("/test/topic")
            .with_compression(&Compression::Png)
            .cmd();
        assert_eq!(cmd.compression, Some("png"));
    }

    #[test]
    fn subscribe() {
        // Test no optional arguments
        let cmd = Subscribe::new("/test/topic");
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(cmd_json, r#"{"op":"subscribe","topic":"/test/topic"}"#);

        // Test all arguments
        let cmd = Subscribe::new("/test/topic")
            .with_id("test")
            .with_type("std_msgs/Example")
            .with_throttle_rate(1)
            .with_queue_length(1)
            .with_fragment_size(1)
            .with_compression(&Compression::None)
            .cmd();
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(
            cmd_json,
            r#"{"op":"subscribe","id":"test","topic":"/test/topic","type":"std_msgs/Example","throttle_rate":1,"queue_length":1,"fragment_size":1,"compression":"none"}"#
        );
    }

    #[test]
    fn unsubscribe() {
        // Test no optional arguments
        let cmd = Unsubscribe::new("/test/topic");
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(cmd_json, r#"{"op":"unsubscribe","topic":"/test/topic"}"#);

        // Test all arguments
        let cmd = Unsubscribe::new("/test/topic").with_id("test").cmd();
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(
            cmd_json,
            r#"{"op":"unsubscribe","id":"test","topic":"/test/topic"}"#
        );
    }

    #[test]
    fn call_service() {
        let test_struct = TestStruct::new();

        // Test no optional arguments
        // We have to type CallService with a bogus serialisable type if providing no arguments
        // This is because serde cannot serialize on the never type <!> operator
        let cmd: CallService<TestStruct> = CallService::new("/test/srv");
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(cmd_json, r#"{"op":"call_service","service":"/test/srv"}"#);

        // Test all arguments
        let cmd = CallService::new("/test/srv")
            .with_id("test")
            .with_args(&test_struct)
            .with_fragment_size(1)
            .with_compression(&Compression::None)
            .cmd();
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(
            cmd_json,
            r#"{"op":"call_service","id":"test","service":"/test/srv","args":{"string":"test string","int":5},"fragment_size":1,"compression":"none"}"#
        );

        // Test deserialise
        let _cmd: CallService<TestStruct> =
            serde_json::from_str(r#"{"op":"call_service","id":"test","service":"/test/srv","args":{"string":"test string","int":5},"fragment_size":1,"compression":"none"}"#)
                .expect("Deserialization failed");
    }

    #[test]
    fn advertise_service() {
        let cmd = AdvertiseService::new("std_srv/Example", "/test/srv");
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(
            cmd_json,
            r#"{"op":"advertise_service","type":"std_srv/Example","service":"/test/srv"}"#
        );
    }

    #[test]
    fn unadvertise_service() {
        let cmd = UnadvertiseService::new("/test/srv");
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(
            cmd_json,
            r#"{"op":"unadvertise_service","service":"/test/srv"}"#
        );
    }

    #[test]
    fn service_response() {
        let test_struct = TestStruct::new();

        // Test no optional arguments
        // We have to type CallService with a bogus serialisable type if providing no arguments
        // This is because serde cannot serialize on the never type <!> operator
        let cmd: ServiceResponse<TestStruct> = ServiceResponse::new("/test/srv", true);
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(
            cmd_json,
            r#"{"op":"service_response","service":"/test/srv","result":true}"#
        );

        // Test all arguments
        let cmd = ServiceResponse::new("/test/srv", true)
            .with_id("test")
            .with_values(&test_struct)
            .cmd();
        let cmd_json = serde_json::to_string(&cmd).expect("Serialization failed");
        assert_eq!(
            cmd_json,
            r#"{"op":"service_response","id":"test","service":"/test/srv","values":{"string":"test string","int":5},"result":true}"#
        );

        // Test deserialise
        let _cmd: ServiceResponse<TestStruct> =
            serde_json::from_str(r#"{"op":"service_response","id":"test","service":"/test/srv","values":{"string":"test string","int":5},"result":true}"#)
                .expect("Deserialization failed");
    }
}
