use serde_derive::{Deserialize, Serialize};
use std::convert::TryFrom;

#[derive(Serialize)]
pub struct Request {}

// Deserialization of an interally tagged enum is currently unsupported by serde-repr
// Combination of tag = "id" and try_from = "u8" is unsupported by serde
// label is never used
// REFERENCE: <https://github.com/dtolnay/serde-repr/issues/3>
#[derive(Deserialize)]
pub struct ResponseMsg {
    pub cancel_response: u8,
}

#[repr(u8)]
#[derive(Deserialize, PartialEq, Debug)]
#[serde(try_from = "ResponseMsg")]
pub enum Response {
    Reject = 1,
    Accept = 2,
}

impl TryFrom<ResponseMsg> for Response {
    type Error = InvalidCancelGoalResponseError;

    fn try_from(response_msg: ResponseMsg) -> Result<Self, Self::Error> {
        if 1 <= response_msg.cancel_response && response_msg.cancel_response <= 2 {
            Ok(unsafe { std::mem::transmute(response_msg.cancel_response) })
        } else {
            Err(InvalidCancelGoalResponseError)
        }
    }
}

#[derive(Debug, Clone)]
pub struct InvalidCancelGoalResponseError;

impl std::fmt::Display for InvalidCancelGoalResponseError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "out of bounds state id does not yeild a cancel goal response")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn response() {
        let response: Response = serde_json::from_str(r#"{"cancel_response":1}"#).unwrap();
        assert_eq!(response, Response::Reject);
    }

    #[test]
    #[should_panic]
    fn response_out_of_bounds() {
        let _response: Response = serde_json::from_str(r#"{"cancel_response":99}"#).unwrap();
    }
}
