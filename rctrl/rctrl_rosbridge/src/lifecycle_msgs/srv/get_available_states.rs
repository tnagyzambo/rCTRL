/// REFERENCE: <https://design.ros2.org/articles/node_lifecycle.html>
/// REFERENCE: <https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/GetAvailableStates.srv>
use serde_derive::Deserialize;

use crate::lifecycle_msgs::msg::State;

#[derive(Deserialize)]
pub struct Response {
    pub available_states: Vec<State>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn response() {
        let _response: Response =
            serde_json::from_str(r#"{"available_states":[{"id":1,"label":"unconfigured"},{"id":1,"label":"unconfigured"},{"id":1,"label":"unconfigured"}]}"#).unwrap();
    }
}
