/// REFERENCE: <https://design.ros2.org/articles/node_lifecycle.html>
/// REFERENCE: <https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/ChangeState.srv>

use serde_derive::{Serialize, Deserialize};

use crate::lifecycle_msgs::msg::Transition;

pub mod change_state {
    use super::*;
    
    #[derive(Serialize)]
    pub struct Request {
        transition: Transition,
    }

    impl Request {
        fn new(transition: Transition) -> Request {
            Request {
                transition: transition,
            }
        }

        pub fn create() -> Request {
            Request::new(Transition::create())
        }

        pub fn configure() -> Request {
            Request::new(Transition::configure())
        }

        pub fn cleanup() -> Request {
            Request::new(Transition::cleanup())
        }

        pub fn activate() -> Request {
            Request::new(Transition::activate())
        }

        pub fn deactivate() -> Request {
            Request::new(Transition::deactivate())
        }

        pub fn shutdown() -> Request {
            Request::new(Transition::active_shutdown())
        }

        pub fn destroy() -> Request {
            Request::new(Transition::destroy())
        }
    }

    #[derive(Deserialize)]
    pub struct Response {
        pub success: bool,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create() {
        let request = change_state::Request::create();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":0,"label":"create"}}"#);
    }

    #[test]
    fn configure() {
        let request = change_state::Request::configure();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":1,"label":"configure"}}"#);
    }

    #[test]
    fn cleanup() {
        let request = change_state::Request::cleanup();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":2,"label":"cleanup"}}"#);
    }

    #[test]
    fn activate() {
        let request = change_state::Request::activate();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":3,"label":"activate"}}"#);
    }

    #[test]
    fn deactivate() {
        let request = change_state::Request::deactivate();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":4,"label":"deactivate"}}"#);
    }

    #[test]
    fn shutdown() {
        let request = change_state::Request::shutdown();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":7,"label":"shutdown"}}"#);
    }

    #[test]
    fn destroy() {
        let request = change_state::Request::destroy();
        let request_json = serde_json::to_string(&request).expect("Serialization failed");
        assert_eq!(request_json, r#"{"transition":{"id":8,"label":"destroy"}}"#);
    }

    #[test]
    fn response() {
        let _response: change_state::Response =
            serde_json::from_str(r#"{"success":true}"#).unwrap();
    }
}
