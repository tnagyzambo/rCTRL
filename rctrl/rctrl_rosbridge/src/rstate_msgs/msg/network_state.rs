use serde_derive::Deserialize;
use std::convert::TryFrom;

// Deserialization of an interally tagged enum is currently unsupported by serde-repr
// Combination of tag = "id" and try_from = "u8" is unsupported by serde
// label is never used
// REFERENCE: <https://github.com/dtolnay/serde-repr/issues/3>
#[derive(Deserialize)]
struct NetworkStateMsg {
    id: u8,
    label: String,
}

#[repr(u8)]
#[derive(Deserialize, PartialEq, Debug)]
#[serde(try_from = "NetworkStateMsg")]
pub enum NetworkState {
    // Primary states
    Unknown = 0,
    Unconfigured = 1,
    Inactive = 2,
    Active = 3,
    Finalized = 4,

    // Transition states
    Configuring = 10,
    CleaningUp = 11,
    Activating = 12,
    Deactivating = 13,
    ShuttingDown = 14,
    ErrorProcessing = 15,
}

impl TryFrom<NetworkStateMsg> for NetworkState {
    type Error = InvalidNetworkStateError;

    fn try_from(state_msg: NetworkStateMsg) -> Result<Self, Self::Error> {
        if state_msg.id <= 5 || (10 <= state_msg.id && state_msg.id <= 17) {
            Ok(unsafe { std::mem::transmute(state_msg.id) })
        } else {
            Err(InvalidNetworkStateError)
        }
    }
}

#[derive(Debug, Clone)]
pub struct InvalidNetworkStateError;

impl std::fmt::Display for InvalidNetworkStateError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "out of bounds state id does not yeild a valid state")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn network_state() {
        let msg: NetworkState = serde_json::from_str(r#"{"id":1,"label":"unconfigured"}"#).unwrap();

        assert_eq!(msg, NetworkState::Unconfigured);
    }

    #[test]
    #[should_panic]
    fn state_out_of_bounds() {
        let _msg: NetworkState = serde_json::from_str(r#"{"id":99,"label":"should fail"}"#).unwrap();
    }
}
