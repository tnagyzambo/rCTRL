/// REFERENCE: <https://design.ros2.org/articles/node_lifecycle.html>
/// REFERENCE: <https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/State.msg>

use std::convert::TryFrom;
use serde_derive::Deserialize;

// Deserialization of an interally tagged enum is currently unsupported by serde-repr
// Combination of tag = "id" and try_from = "u8" is unsupported by serde
// label is never used
// REFERENCE: <https://github.com/dtolnay/serde-repr/issues/3>
#[derive(Deserialize)]
struct StateMsg {
    id: u8,
    // label: String,
}

#[repr(u8)]
#[derive(Deserialize, PartialEq, Debug)]
#[serde(try_from = "StateMsg")]
pub enum State {
    // These are the primary states. State changes can only be requested when the node is in one of these states.

    // # Indicates state has not yet been set.
    Unknown = 0,
    
    // This is the life cycle state the node is in immediately after being instantiated.
    Unconfigured = 1,

    // This state represents a node that is not currently performing any processing.
    Inactive = 2,

    // This is the main state of the node’s life cycle. While in this state, the node
    // performs any processing, responds to service requests, reads and processes
    // data, produces output, etc.
    Active = 3,

    // The finalized state is the state in which the node ends immediately before being destroyed.
    Finalized = 4,

    // Temporary intermediate states. When a transition is requested, the node changes its state into one of these states.

    // In this transition state the node’s onConfigure callback will be called to
    // allow the node to load its configuration and conduct any required setup.
    Configuring = 10,

    // In this transition state the node’s callback onCleanup will be called to clear
    // all state and return the node to a functionally equivalent state as when first created.
    CleaningUp = 11,

    // In this transition state the callback onShutdown will be executed to do any cleanup necessary before destruction.
    ShuttingDown = 12,

    // In this transition state the callback onActivate will be executed to do any final preparations to start executing.
    Activating = 13,

    // In this transition state the callback onDeactivate will be executed to do any
    // cleanup to start executing, and reverse the onActivate changes.
    Deactivating = 14,

    // This transition state is where any error may be cleaned up.
    ErrorProcessing = 15,
}

impl TryFrom<StateMsg> for State {
    type Error = InvalidStateError;

    fn try_from(state_msg: StateMsg) -> Result<Self, Self::Error> {
        if state_msg.id <= 4 || (10 <= state_msg.id && state_msg.id <= 15) {
            Ok(unsafe { std::mem::transmute(state_msg.id) })
        } else {
            Err(InvalidStateError)
        }
    }
}

#[derive(Debug, Clone)]
pub struct InvalidStateError;

impl std::fmt::Display for InvalidStateError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "out of bounds state id does not yeild a valid state")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state() {
        let msg: State =
            serde_json::from_str(r#"{"id":1,"label":"unconfigured"}"#).unwrap();

        assert_eq!(msg, State::Unconfigured);
    }

    #[test]
    #[should_panic]
    fn state_out_of_bounds() {
        let _msg: State =
            serde_json::from_str(r#"{"id":99,"label":"should fail"}"#).unwrap();
    }
}
