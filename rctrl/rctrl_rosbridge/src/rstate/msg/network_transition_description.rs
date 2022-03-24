use serde_derive::Deserialize;

use crate::rstate::msg::NetworkState;
use crate::rstate::msg::NetworkTransition;

#[derive(Deserialize, PartialEq, Debug)]
pub struct NetworkTransitionDescription {
    pub transition: NetworkTransition,
    pub start_state: NetworkState,
    pub goal_state: NetworkState,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn network_transition_description() {
        let msg: NetworkTransitionDescription =
            serde_json::from_str(r#"{"transition":{"id":0,"label":"configure"},"start_state":{"id":1,"label":"unconfigured"},"goal_state":{"id":2,"label":"inactive"}}"#).unwrap();

        let network_transition_description = NetworkTransitionDescription {
            transition: NetworkTransition::Configure,
            start_state: NetworkState::Unconfigured,
            goal_state: NetworkState::Inactive,
        };

        assert_eq!(msg, network_transition_description);
    }
}
