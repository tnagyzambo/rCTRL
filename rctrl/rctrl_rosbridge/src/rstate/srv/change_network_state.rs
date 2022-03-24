use serde_derive::{Deserialize, Serialize};
use std::convert::From;

use crate::rstate::msg::NetworkTransition;

#[derive(Serialize)]
pub struct Request {
    transition: NetworkTransition,
}

impl From<NetworkTransition> for Request {
    fn from(transition: NetworkTransition) -> Self {
        Request { transition: transition }
    }
}

#[derive(Deserialize)]
pub struct Response {
    pub success: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn configure() {
        let request = Request::from(NetworkTransition::Configure);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":0,"label":"configure"}}"#);
    }

    #[test]
    fn cleanup() {
        let request = Request::from(NetworkTransition::CleanUp);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":1,"label":"cleanup"}}"#);
    }

    #[test]
    fn activate() {
        let request = Request::from(NetworkTransition::Activate);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":2,"label":"activate"}}"#);
    }

    #[test]
    fn deactivate() {
        let request = Request::from(NetworkTransition::Deactivate);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":3,"label":"deactivate"}}"#);
    }

    #[test]
    fn arm() {
        let request = Request::from(NetworkTransition::Arm);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":4,"label":"arm"}}"#);
    }

    #[test]
    fn disarm() {
        let request = Request::from(NetworkTransition::Disarm);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":5,"label":"disarm"}}"#);
    }

    #[test]
    fn shutdown() {
        let request = Request::from(NetworkTransition::Shutdown);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":6,"label":"shutdown"}}"#);
    }

    #[test]
    fn response() {
        let _response: Response = serde_json::from_str(r#"{"success":true}"#).unwrap();
    }
}
