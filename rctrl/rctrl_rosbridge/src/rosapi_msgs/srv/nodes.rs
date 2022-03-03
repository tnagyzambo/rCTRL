/// REFERENCE: <https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/rosapi_msgs/srv/Nodes.srv>
use serde_derive::Deserialize;

#[derive(Deserialize)]
pub struct Response {
    pub nodes: Vec<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn response() {
        let _response: Response =
            serde_json::from_str(r#"{"nodes":["/rdata", "rosapi"]}"#).unwrap();
    }
}
