/// REFERENCE: <https://design.ros2.org/articles/node_lifecycle.html>
/// REFERENCE: <https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/GetAvailableTransitions.srv>
use serde_derive::Deserialize;

use crate::lifecycle_msgs::msg::TransitionDescription;

#[derive(Deserialize)]
pub struct Response {
    pub available_transitions: Vec<TransitionDescription>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn response() {
        let _response: Response =
            serde_json::from_str(r#"{"available_transitions":[{"transition":{"id":1,"label":"configure"},"start_state":{"id":1,"label":"unconfigured"},"goal_state":{"id":2,"label":"inactive"}},{"transition":{"id":1,"label":"configure"},"start_state":{"id":1,"label":"unconfigured"},"goal_state":{"id":2,"label":"inactive"}},{"transition":{"id":1,"label":"configure"},"start_state":{"id":1,"label":"unconfigured"},"goal_state":{"id":2,"label":"inactive"}}]}"#).unwrap();
    }
}
