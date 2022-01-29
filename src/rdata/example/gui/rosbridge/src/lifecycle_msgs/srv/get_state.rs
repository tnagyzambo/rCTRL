/// REFERENCE: <https://design.ros2.org/articles/node_lifecycle.html>
/// REFERENCE: <https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/GetState.srv>

use serde_derive::Deserialize;

use crate::lifecycle_msgs::msg::State;

pub mod get_state {
    use super::*;

    #[derive(Deserialize)]
    pub struct Response {
        pub current_state: State,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn response() {
        let _response: get_state::Response =
            serde_json::from_str(r#"{"current_state":{"id":1,"label":"unconfigured"}}"#).unwrap();
    }
}
