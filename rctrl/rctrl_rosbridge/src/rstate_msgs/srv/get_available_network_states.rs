use serde_derive::Deserialize;

use crate::rstate_msgs::msg::NetworkState;

#[derive(Deserialize)]
pub struct Response {
    pub available_states: Vec<NetworkState>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn response() {
        let _response: Response = serde_json::from_str(
            r#"{"available_states":[{"id":0,"label":"unconfigured"},{"id":0,"label":"unconfigured"},{"id":0,"label":"unconfigured"}]}"#,
        )
        .unwrap();
    }
}
