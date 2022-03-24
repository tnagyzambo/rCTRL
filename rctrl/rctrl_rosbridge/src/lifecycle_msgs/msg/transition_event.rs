/// REFERENCE: <https://design.ros2.org/articles/node_lifecycle.html>
/// REFERENCE: <https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/TransitionEvent.msg>
use serde_derive::Deserialize;

use crate::lifecycle_msgs::msg::State;
use crate::lifecycle_msgs::msg::Transition;

#[derive(Deserialize, PartialEq, Debug)]
pub struct TransitionEvent {
    pub timestamp: u64,
    pub transition: Transition,
    pub start_state: State,
    pub goal_state: State,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transition_event() {
        let msg: TransitionEvent =
            serde_json::from_str(r#"{"timestamp":100000,"transition":{"id":1,"label":"configure"},"start_state":{"id":1,"label":"unconfigured"},"goal_state":{"id":2,"label":"inactive"}}"#).unwrap();

        let transition_event = TransitionEvent {
            timestamp: 100000,
            transition: Transition::Configure,
            start_state: State::Unconfigured,
            goal_state: State::Inactive,
        };

        assert_eq!(msg, transition_event);
    }
}
