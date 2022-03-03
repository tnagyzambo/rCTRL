/// REFERENCE: <https://design.ros2.org/articles/node_lifecycle.html>
/// REFERENCE: <https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/ChangeState.srv>
use serde_derive::{Deserialize, Serialize};
use std::convert::From;

use crate::lifecycle_msgs::msg::Transition;

#[derive(Serialize)]
pub struct Request {
    transition: Transition,
}

impl From<Transition> for Request {
    fn from(transition: Transition) -> Self {
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
    fn create() {
        let request = Request::from(Transition::Create);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":0,"label":"create"}}"#);
    }

    #[test]
    fn configure() {
        let request = Request::from(Transition::Configure);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":1,"label":"configure"}}"#);
    }

    #[test]
    fn cleanup() {
        let request = Request::from(Transition::CleanUp);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":2,"label":"cleanup"}}"#);
    }

    #[test]
    fn activate() {
        let request = Request::from(Transition::Activate);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":3,"label":"activate"}}"#);
    }

    #[test]
    fn deactivate() {
        let request = Request::from(Transition::Deactivate);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":4,"label":"deactivate"}}"#);
    }

    #[test]
    fn unconfigured_shutdown() {
        let request = Request::from(Transition::UnconfiguredShutdown);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":5,"label":"shutdown"}}"#);
    }

    #[test]
    fn inactive_shutdown() {
        let request = Request::from(Transition::InactiveShutdown);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":6,"label":"shutdown"}}"#);
    }

    #[test]
    fn active_shutdown() {
        let request = Request::from(Transition::ActiveShutdown);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":7,"label":"shutdown"}}"#);
    }

    #[test]
    fn destroy() {
        let request = Request::from(Transition::Destroy);
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":8,"label":"destroy"}}"#);
    }

    #[test]
    fn response() {
        let _response: Response = serde_json::from_str(r#"{"success":true}"#).unwrap();
    }
}
