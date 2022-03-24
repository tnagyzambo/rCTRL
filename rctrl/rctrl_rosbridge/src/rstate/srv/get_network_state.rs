use serde_derive::Deserialize;

use crate::rstate::msg::NetworkState;

#[derive(Deserialize)]
pub struct Response {
    pub current_state: NetworkState,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn response() {
        let _response: Response = serde_json::from_str(r#"{"current_state":{"id":0,"label":"unconfigured"}}"#).unwrap();
    }
}
