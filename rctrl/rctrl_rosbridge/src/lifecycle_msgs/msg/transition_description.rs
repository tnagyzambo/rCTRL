/// REFERENCE: <https://design.ros2.org/articles/node_lifecycle.html>
/// REFERENCE: <https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/TransitionDescription.msg>
use serde_derive::Deserialize;

use crate::lifecycle_msgs::msg::State;
use crate::lifecycle_msgs::msg::Transition;

#[derive(Deserialize, PartialEq, Debug)]
pub struct TransitionDescription {
    pub transition: Transition,
    pub start_state: State,
    pub goal_state: State,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transition_description() {
        let msg: TransitionDescription =
            serde_json::from_str(r#"{"transition":{"id":1,"label":"configure"},"start_state":{"id":1,"label":"unconfigured"},"goal_state":{"id":2,"label":"inactive"}}"#).unwrap();

        let transition_description = TransitionDescription {
            transition: Transition::Configure,
            start_state: State::Unconfigured,
            goal_state: State::Inactive,
        };

        assert_eq!(msg, transition_description);
    }
}
