use serde::Serialize;

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
