use serde_derive::Deserialize;

use crate::rstate_msgs::msg::NetworkTransitionDescription;

#[derive(Deserialize)]
pub struct Response {
    pub available_transitions: Vec<NetworkTransitionDescription>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn response() {
        let _response: Response =
            serde_json::from_str(r#"{"available_transitions":[{"transition":{"id":0,"label":"configure"},"start_state":{"id":1,"label":"unconfigured"},"goal_state":{"id":2,"label":"inactive"}},{"transition":{"id":0,"label":"configure"},"start_state":{"id":1,"label":"unconfigured"},"goal_state":{"id":2,"label":"inactive"}},{"transition":{"id":0,"label":"configure"},"start_state":{"id":1,"label":"unconfigured"},"goal_state":{"id":2,"label":"inactive"}}]}"#).unwrap();
    }
}
