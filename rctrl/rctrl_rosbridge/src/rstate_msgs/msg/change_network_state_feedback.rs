use serde_derive::Deserialize;

#[derive(Deserialize, PartialEq, Debug)]
pub struct ChangeNetworkStateFeedback {
    pub cmd_total: u8,
    pub cmd_complete: u8,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn network_transition_feedback() {
        let feedback = ChangeNetworkStateFeedback {
            cmd_total: 1,
            cmd_complete: 1,
        };

        let msg: ChangeNetworkStateFeedback = serde_json::from_str(r#"{"cmd_total":1,"cmd_complete":1}"#).unwrap();

        assert_eq!(msg, feedback);
    }
}
