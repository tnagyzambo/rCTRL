/// REFERENCE: <https://design.ros2.org/articles/node_lifecycle.html>
/// REFERENCE: <https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/ChangeState.srv>

use serde_derive::{Serialize, Deserialize};

use crate::lifecycle_msgs::msg::transition::Transition;

#[derive(Serialize)]
pub struct ChangeStateRequest<'a> {
    transition: Transition<'a>,
}

impl<'a> ChangeStateRequest<'a> {
    fn new(transition: Transition<'a>) -> ChangeStateRequest<'a> {
        ChangeStateRequest {
            transition: transition,
        }
    }

    pub fn create() -> ChangeStateRequest<'a> {
        ChangeStateRequest::new(Transition::create())
    }

    pub fn configure() -> ChangeStateRequest<'a> {
        ChangeStateRequest::new(Transition::configure())
    }

    pub fn cleanup() -> ChangeStateRequest<'a> {
        ChangeStateRequest::new(Transition::cleanup())
    }

    pub fn activate() -> ChangeStateRequest<'a> {
        ChangeStateRequest::new(Transition::activate())
    }

    pub fn deactivate() -> ChangeStateRequest<'a> {
        ChangeStateRequest::new(Transition::deactivate())
    }

    pub fn shutdown() -> ChangeStateRequest<'a> {
        ChangeStateRequest::new(Transition::active_shutdown())
    }

    pub fn destroy() -> ChangeStateRequest<'a> {
        ChangeStateRequest::new(Transition::destroy())
    }
}

#[derive(Deserialize)]
pub struct ChangeStateResponse {
    success: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create() {
        let request = ChangeStateRequest::create();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":0,"label":"create"}}"#);
    }

    #[test]
    fn configure() {
        let request = ChangeStateRequest::configure();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":1,"label":"configure"}}"#);
    }

    #[test]
    fn cleanup() {
        let request = ChangeStateRequest::cleanup();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":2,"label":"cleanup"}}"#);
    }

    #[test]
    fn activate() {
        let request = ChangeStateRequest::activate();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":3,"label":"activate"}}"#);
    }

    #[test]
    fn deactivate() {
        let request = ChangeStateRequest::deactivate();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":4,"label":"deactivate"}}"#);
    }

    #[test]
    fn shutdown() {
        let request = ChangeStateRequest::shutdown();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":7,"label":"shutdown"}}"#);
    }

    #[test]
    fn destroy() {
        let request = ChangeStateRequest::destroy();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":8,"label":"destroy"}}"#);
    }

    #[test]
    fn response() {
        let _response: ChangeStateResponse =
            serde_json::from_str(r#"{"success":true}"#).expect("Deserialization failed");
    }
}
