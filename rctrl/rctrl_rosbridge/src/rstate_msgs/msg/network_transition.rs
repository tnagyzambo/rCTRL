use serde_derive::{Deserialize, Serialize};
use std::convert::{Into, TryFrom};

#[derive(Serialize, Deserialize, Eq, PartialEq, Hash, Debug)]
struct NetworkTransitionMsg {
    id: u8,
    label: String,
}

#[repr(u8)]
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[serde(into = "NetworkTransitionMsg", try_from = "NetworkTransitionMsg")]
pub enum NetworkTransition {
    Configure = 0,
    CleanUp = 1,
    Activate = 2,
    Deactivate = 3,
    Shutdown = 4,
}

impl NetworkTransition {
    pub fn label(&self) -> &'static str {
        match self {
            NetworkTransition::Configure => "configure",
            NetworkTransition::CleanUp => "cleanup",
            NetworkTransition::Activate => "activate",
            NetworkTransition::Deactivate => "deactivate",
            NetworkTransition::Shutdown => "shutdown",
        }
    }
}

impl std::fmt::Display for NetworkTransition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let label = match self {
            NetworkTransition::Configure => "Configure",
            NetworkTransition::CleanUp => "Clean Up",
            NetworkTransition::Activate => "Activate",
            NetworkTransition::Deactivate => "Deactivate",
            NetworkTransition::Shutdown => "Shutdown",
        };
        write!(f, "{}", label)
    }
}

impl Into<NetworkTransitionMsg> for NetworkTransition {
    fn into(self) -> NetworkTransitionMsg {
        let label = self.label().to_string();

        NetworkTransitionMsg {
            id: (self as u8),
            label: label,
        }
    }
}

impl TryFrom<NetworkTransitionMsg> for NetworkTransition {
    type Error = InvalidNetworkTransitionError;

    fn try_from(transition_msg: NetworkTransitionMsg) -> Result<Self, Self::Error> {
        if transition_msg.id <= 6 {
            Ok(unsafe { std::mem::transmute(transition_msg.id) })
        } else {
            Err(InvalidNetworkTransitionError)
        }
    }
}

#[derive(Debug, Clone)]
pub struct InvalidNetworkTransitionError;

impl std::fmt::Display for InvalidNetworkTransitionError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "out of bounds transition id does not yeild a valid transition")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn configure() {
        let transition = NetworkTransition::Configure;
        let transition_json = serde_json::to_string(&transition).unwrap();
        assert_eq!(transition_json, r#"{"id":0,"label":"configure"}"#);

        let _transition_deserialize: NetworkTransitionMsg = serde_json::from_str(&transition_json).unwrap();
    }

    #[test]
    fn cleanup() {
        let transition = NetworkTransition::CleanUp;
        let transition_json = serde_json::to_string(&transition).unwrap();
        assert_eq!(transition_json, r#"{"id":1,"label":"cleanup"}"#);

        let _transition_deserialize: NetworkTransitionMsg = serde_json::from_str(&transition_json).unwrap();
    }

    #[test]
    fn activate() {
        let transition = NetworkTransition::Activate;
        let transition_json = serde_json::to_string(&transition).unwrap();
        assert_eq!(transition_json, r#"{"id":2,"label":"activate"}"#);

        let _transition_deserialize: NetworkTransitionMsg = serde_json::from_str(&transition_json).unwrap();
    }

    #[test]
    fn deactivate() {
        let transition = NetworkTransition::Deactivate;
        let transition_json = serde_json::to_string(&transition).unwrap();
        assert_eq!(transition_json, r#"{"id":3,"label":"deactivate"}"#);

        let _transition_deserialize: NetworkTransitionMsg = serde_json::from_str(&transition_json).unwrap();
    }

    #[test]
    fn shutdown() {
        let transition = NetworkTransition::Shutdown;
        let transition_json = serde_json::to_string(&transition).unwrap();
        assert_eq!(transition_json, r#"{"id":6,"label":"shutdown"}"#);

        let _transition_deserialize: NetworkTransitionMsg = serde_json::from_str(&transition_json).unwrap();
    }

    #[test]
    #[should_panic]
    fn transition_out_of_bounds() {
        let _msg: NetworkTransition = serde_json::from_str(r#"{"id":99,"label":"should fail"}"#).unwrap();
    }
}
