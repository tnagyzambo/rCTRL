use serde::Serialize;

#[derive(Serialize)]
pub struct Msg<'a> {
    transition: &'a Transition<'a>,
}

impl<'a> Msg<'a> {
    pub fn new(transition: &'a Transition) -> Msg<'a> {
        Msg {
            transition: transition,
        }
    }
}

#[derive(Serialize)]
pub struct Transition<'a> {
    id: u8,
    label: &'a str,
}

impl<'a> Transition<'a> {
    pub fn new(id: u8, label: &'a str) -> Transition<'a> {
        Transition {
            id: id,
            label: label,
        }
    }
}
