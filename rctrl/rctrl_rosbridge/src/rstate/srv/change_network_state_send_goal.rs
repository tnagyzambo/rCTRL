use serde_derive::{Deserialize, Serialize};
use std::convert::From;
use std::convert::TryFrom;

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

// Deserialization of an interally tagged enum is currently unsupported by serde-repr
// Combination of tag = "id" and try_from = "u8" is unsupported by serde
// label is never used
// REFERENCE: <https://github.com/dtolnay/serde-repr/issues/3>
#[derive(Deserialize)]
pub struct ResponseMsg {
    pub goal_response: u8,
}

#[repr(u8)]
#[derive(Deserialize, PartialEq, Debug)]
#[serde(try_from = "ResponseMsg")]
pub enum Response {
    Reject = 1,
    AcceptAndExecute = 2,
    AcceptAndDefer = 3,
}

impl TryFrom<ResponseMsg> for Response {
    type Error = InvalidGoalResponseError;

    fn try_from(response_msg: ResponseMsg) -> Result<Self, Self::Error> {
        if 1 <= response_msg.goal_response && response_msg.goal_response <= 3 {
            Ok(unsafe { std::mem::transmute(response_msg.goal_response) })
        } else {
            Err(InvalidGoalResponseError)
        }
    }
}

#[derive(Debug, Clone)]
pub struct InvalidGoalResponseError;

impl std::fmt::Display for InvalidGoalResponseError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "out of bounds state id does not yeild a goal response")
    }
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
        let response: Response = serde_json::from_str(r#"{"goal_response":1}"#).unwrap();
        assert_eq!(response, Response::Reject);
    }

    #[test]
    #[should_panic]
    fn response_out_of_bounds() {
        let _response: Response = serde_json::from_str(r#"{"goal_response":99}"#).unwrap();
    }
}
