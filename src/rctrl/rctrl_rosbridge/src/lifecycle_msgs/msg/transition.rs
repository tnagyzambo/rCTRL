/// REFERENCE: <https://design.ros2.org/articles/node_lifecycle.html>
/// REFERENCE: <https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/Transition.msg>
use serde_derive::{Deserialize, Serialize};
use std::convert::{Into, TryFrom};

#[derive(Serialize, Deserialize, Eq, PartialEq, Hash, Debug)]
struct TransitionMsg {
    id: u8,
    label: String,
}

#[repr(u8)]
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[serde(into = "TransitionMsg", try_from = "TransitionMsg")]
pub enum Transition {
    // This transition will instantiate the node, but will not run any code beyond
    // the constructor.
    Create = 0,

    // The node’s onConfigure callback will be called to allow the node to load its
    // configuration and conduct any required setup.
    Configure = 1,

    // The node’s callback onCleanup will be called in this transition to allow the
    // node to load its configuration and conduct any required setup.
    CleanUp = 2,

    // The node's callback onActivate will be executed to do any final preparations
    // to start executing.
    Activate = 3,

    // The node's callback onDeactivate will be executed to do any cleanup to start
    // executing, and reverse the onActivate changes.
    Deactivate = 4,

    // These are specified in the lifecycle msgs code but I don't see them used in practice
    // They also conflict with the ROS2 design docs
    //
    // I'm pretty sure that these three different shutdowns are all treated equally
    // Looking at the ROS2 lifecycle design doc there is only one possible shutdown event
    // This signals shutdown during an unconfigured state, the node's callback
    // onShutdown will be executed to do any cleanup necessary before destruction.
    // UnconfiguredShutdown = 5,

    // This signals shutdown during an inactive state, the node's callback onShutdown
    // will be executed to do any cleanup necessary before destruction.
    // InactiveShutdown = 6,

    // This signals shutdown during an active state, the node's callback onShutdown
    // will be executed to do any cleanup necessary before destruction.
    // ActiveShutdown = 7,

    // This seems to reflect reality
    Shutdown = 5,

    // This transition will simply cause the deallocation of the node.
    Destroy = 8,
}

impl Transition {
    fn label(&self) -> &'static str {
        match self {
            Transition::Create => "create",
            Transition::Configure => "configure",
            Transition::CleanUp => "cleanup",
            Transition::Activate => "activate",
            Transition::Deactivate => "deactivate",
            Transition::Shutdown => "shutdown",
            Transition::Destroy => "destroy",
        }
    }
}

impl Into<TransitionMsg> for Transition {
    fn into(self) -> TransitionMsg {
        let label = self.label().to_string();

        TransitionMsg {
            id: (self as u8),
            label: label,
        }
    }
}

impl TryFrom<TransitionMsg> for Transition {
    type Error = InvalidTransitionError;

    fn try_from(transition_msg: TransitionMsg) -> Result<Self, Self::Error> {
        if transition_msg.id <= 5 || transition_msg.id == 8 {
            Ok(unsafe { std::mem::transmute(transition_msg.id) })
        } else {
            Err(InvalidTransitionError)
        }
    }
}

#[derive(Debug, Clone)]
pub struct InvalidTransitionError;

impl std::fmt::Display for InvalidTransitionError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "out of bounds transition id does not yeild a valid transition")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create() {
        let transition = Transition::Create;
        let transition_json = serde_json::to_string(&transition).unwrap();
        assert_eq!(transition_json, r#"{"id":0,"label":"create"}"#);

        let _transition_deserialize: Transition = serde_json::from_str(&transition_json).unwrap();
    }

    #[test]
    fn configure() {
        let transition = Transition::Configure;
        let transition_json = serde_json::to_string(&transition).unwrap();
        assert_eq!(transition_json, r#"{"id":1,"label":"configure"}"#);

        let _transition_deserialize: Transition = serde_json::from_str(&transition_json).unwrap();
    }

    #[test]
    fn cleanup() {
        let transition = Transition::CleanUp;
        let transition_json = serde_json::to_string(&transition).unwrap();
        assert_eq!(transition_json, r#"{"id":2,"label":"cleanup"}"#);

        let _transition_deserialize: Transition = serde_json::from_str(&transition_json).unwrap();
    }

    #[test]
    fn activate() {
        let transition = Transition::Activate;
        let transition_json = serde_json::to_string(&transition).unwrap();
        assert_eq!(transition_json, r#"{"id":3,"label":"activate"}"#);

        let _transition_deserialize: Transition = serde_json::from_str(&transition_json).unwrap();
    }

    #[test]
    fn deactivate() {
        let transition = Transition::Deactivate;
        let transition_json = serde_json::to_string(&transition).unwrap();
        assert_eq!(transition_json, r#"{"id":4,"label":"deactivate"}"#);

        let _transition_deserialize: Transition = serde_json::from_str(&transition_json).unwrap();
    }

    // #[test]
    // fn unconfigured_shutdown() {
    //     let transition = Transition::UnconfiguredShutdown;
    //     let transition_json = serde_json::to_string(&transition).unwrap();
    //     assert_eq!(transition_json, r#"{"id":5,"label":"shutdown"}"#);

    //     let _transition_deserialize: Transition = serde_json::from_str(&transition_json).unwrap();
    // }

    // #[test]
    // fn inactive_shutdown() {
    //     let transition = Transition::InactiveShutdown;
    //     let transition_json = serde_json::to_string(&transition).unwrap();
    //     assert_eq!(transition_json, r#"{"id":6,"label":"shutdown"}"#);

    //     let _transition_deserialize: Transition = serde_json::from_str(&transition_json).unwrap();
    // }

    // #[test]
    // fn active_shutdown() {
    //     let transition = Transition::ActiveShutdown;
    //     let transition_json = serde_json::to_string(&transition).unwrap();
    //     assert_eq!(transition_json, r#"{"id":7,"label":"shutdown"}"#);

    //     let _transition_deserialize: Transition = serde_json::from_str(&transition_json).unwrap();
    // }

    #[test]
    fn unconfigured_shutdown() {
        let transition = Transition::Shutdown;
        let transition_json = serde_json::to_string(&transition).unwrap();
        assert_eq!(transition_json, r#"{"id":5,"label":"shutdown"}"#);

        let _transition_deserialize: Transition = serde_json::from_str(&transition_json).unwrap();
    }

    #[test]
    fn destroy() {
        let transition = Transition::Destroy;
        let transition_json = serde_json::to_string(&transition).unwrap();
        assert_eq!(transition_json, r#"{"id":8,"label":"destroy"}"#);

        let _transition_deserialize: Transition = serde_json::from_str(&transition_json).unwrap();
    }

    #[test]
    #[should_panic]
    fn transition_out_of_bounds() {
        let _msg: Transition = serde_json::from_str(r#"{"id":99,"label":"should fail"}"#).unwrap();
    }
}
