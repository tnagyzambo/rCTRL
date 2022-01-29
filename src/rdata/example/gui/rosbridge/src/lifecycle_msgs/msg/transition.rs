/// REFERENCE: <https://design.ros2.org/articles/node_lifecycle.html>
/// REFERENCE: <https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/Transition.msg>

use serde_derive::Serialize;

#[derive(Serialize)]
pub struct Transition<'a> {
    id: u8,
    label: &'a str,
}

impl<'a> Transition<'a> {
    fn new(id: u8, label: &'a str) -> Transition<'a> {
        Transition {
            id: id,
            label: label,
        }
    }

    // This transition will instantiate the node, but will not run any code beyond
    // the constructor.
    pub fn create() -> Transition<'a> {
        Transition::new(0, "create")
    }

    // The node’s onConfigure callback will be called to allow the node to load its
    // configuration and conduct any required setup.
    pub fn configure() -> Transition<'a> {
        Transition::new(1, "configure")
    }

    // The node’s callback onCleanup will be called in this transition to allow the
    // node to load its configuration and conduct any required setup.
    pub fn cleanup() -> Transition<'a> {
        Transition::new(2, "cleanup")
    }

    // The node's callback onActivate will be executed to do any final preparations
    // to start executing.
    pub fn activate() -> Transition<'a> {
        Transition::new(3, "activate")
    }

    // The node's callback onDeactivate will be executed to do any cleanup to start
    // executing, and reverse the onActivate changes.
    pub fn deactivate() -> Transition<'a> {
        Transition::new(4, "deactivate")
    }

    // I'm pretty sure that these three different shutdowns are all treated equally
    // Looking at the ROS2 lifecycle design doc there is only one possible shutdown event
    // This signals shutdown during an unconfigured state, the node's callback
    // onShutdown will be executed to do any cleanup necessary before destruction.
    pub fn unconfigured_shutdown() -> Transition<'a> {
        Transition::new(5, "shutdown")
    }

    // This signals shutdown during an inactive state, the node's callback onShutdown
    // will be executed to do any cleanup necessary before destruction.
    pub fn inactive_shutdown() -> Transition<'a> {
        Transition::new(6, "shutdown")
    }

    // This signals shutdown during an active state, the node's callback onShutdown
    // will be executed to do any cleanup necessary before destruction.
    pub fn active_shutdown() -> Transition<'a> {
        Transition::new(7, "shutdown")
    }

    // This transition will simply cause the deallocation of the node.
    pub fn destroy() -> Transition<'a> {
        Transition::new(8, "destroy")
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;

    #[test]
    fn create() {
        let transition = Transition::create();
        let transition_json = serde_json::to_string(&transition).expect("Serialization failed");
        assert_eq!(transition_json, r#"{"id":0,"label":"create"}"#);
    }

    #[test]
    fn configure() {
        let transition = Transition::configure();
        let transition_json = serde_json::to_string(&transition).expect("Serialization failed");
        assert_eq!(transition_json, r#"{"id":1,"label":"configure"}"#);
    }

    #[test]
    fn cleanup() {
        let transition = Transition::cleanup();
        let transition_json = serde_json::to_string(&transition).expect("Serialization failed");
        assert_eq!(transition_json, r#"{"id":2,"label":"cleanup"}"#);
    }

    #[test]
    fn activate() {
        let transition = Transition::activate();
        let transition_json = serde_json::to_string(&transition).expect("Serialization failed");
        assert_eq!(transition_json, r#"{"id":3,"label":"activate"}"#);
    }

    #[test]
    fn deactivate() {
        let transition = Transition::deactivate();
        let transition_json = serde_json::to_string(&transition).expect("Serialization failed");
        assert_eq!(transition_json, r#"{"id":4,"label":"deactivate"}"#);
    }

    #[test]
    fn unconfigured_shutdown() {
        let transition = Transition::unconfigured_shutdown();
        let transition_json = serde_json::to_string(&transition).expect("Serialization failed");
        assert_eq!(transition_json, r#"{"id":5,"label":"shutdown"}"#);
    }

    #[test]
    fn inactive_shutdown() {
        let transition = Transition::inactive_shutdown();
        let transition_json = serde_json::to_string(&transition).expect("Serialization failed");
        assert_eq!(transition_json, r#"{"id":6,"label":"shutdown"}"#);
    }

    #[test]
    fn active_shutdown() {
        let transition = Transition::active_shutdown();
        let transition_json = serde_json::to_string(&transition).expect("Serialization failed");
        assert_eq!(transition_json, r#"{"id":7,"label":"shutdown"}"#);
    }

    #[test]
    fn destroy() {
        let transition = Transition::destroy();
        let transition_json = serde_json::to_string(&transition).expect("Serialization failed");
        assert_eq!(transition_json, r#"{"id":8,"label":"destroy"}"#);
    }
}
