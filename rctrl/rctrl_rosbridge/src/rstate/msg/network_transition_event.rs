/// REFERENCE: <https://design.ros2.org/articles/node_lifecycle.html>
/// REFERENCE: <https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/TransitionEvent.msg>
use serde_derive::Deserialize;

use crate::rstate::msg::NetworkState;
use crate::rstate::msg::NetworkTransition;

#[derive(Deserialize, PartialEq, Debug)]
pub struct NetworkTransitionEvent {
    pub timestamp: u64,
    pub transition: NetworkTransition,
    pub start_state: NetworkState,
    pub goal_state: NetworkState,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn network_transition_event() {
        let msg: NetworkTransitionEvent =
            serde_json::from_str(r#"{"timestamp":100000,"transition":{"id":0,"label":"configure"},"start_state":{"id":1,"label":"unconfigured"},"goal_state":{"id":2,"label":"inactive"}}"#).unwrap();

        let network_transition_event = NetworkTransitionEvent {
            timestamp: 100000,
            transition: NetworkTransition::Configure,
            start_state: NetworkState::Unconfigured,
            goal_state: NetworkState::Inactive,
        };

        assert_eq!(msg, network_transition_event);
    }
}
