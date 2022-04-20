use serde_derive::Deserialize;
use std::convert::TryFrom;

// Deserialization of an interally tagged enum is currently unsupported by serde-repr
// Combination of tag = "id" and try_from = "u8" is unsupported by serde
// label is never used
// REFERENCE: <https://github.com/dtolnay/serde-repr/issues/3>
#[derive(Deserialize)]
struct ValveStateMsg {
    id: u8,
    label: String,
}

#[repr(u8)]
#[derive(Deserialize, PartialEq, Debug)]
#[serde(try_from = "ValveStateMsg")]
pub enum ValveState {
    // Primary states
    Closed = 0,
    Open = 1,
    Unknown = 2,
}

impl TryFrom<ValveStateMsg> for ValveState {
    type Error = InvalidValveStateError;

    fn try_from(state_msg: ValveStateMsg) -> Result<Self, Self::Error> {
        if state_msg.id <= 5 || (10 <= state_msg.id && state_msg.id <= 17) {
            Ok(unsafe { std::mem::transmute(state_msg.id) })
        } else {
            Err(InvalidValveStateError)
        }
    }
}

#[derive(Debug, Clone)]
pub struct InvalidValveStateError;

impl std::fmt::Display for InvalidValveStateError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "out of bounds state id does not yeild a valid state")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valve_state() {
        let msg: ValveState = serde_json::from_str(r#"{"id":1,"label":"open"}"#).unwrap();

        assert_eq!(msg, ValveState::Open);
    }

    #[test]
    #[should_panic]
    fn state_out_of_bounds() {
        let _msg: ValveState = serde_json::from_str(r#"{"id":99,"label":"should fail"}"#).unwrap();
    }
}
