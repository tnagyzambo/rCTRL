use serde_derive::Serialize;

use crate::lifecycle_msgs::msg::transition::Transition;

#[derive(Serialize)]
pub struct ChangeStateRequest<'a> {
    transition: &'a Transition<'a>,
}

impl<'a> ChangeStateRequest<'a> {
    pub fn new(transition: &'a Transition) -> ChangeStateRequest<'a> {
        ChangeStateRequest {
            transition: transition,
        }
    }
}
