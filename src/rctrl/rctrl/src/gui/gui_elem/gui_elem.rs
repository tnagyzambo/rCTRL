/// REFERENCE: <https://stackoverflow.com/questions/45786717/how-to-implement-hashmap-with-two-keys/45795699>
use eframe::egui;
use serde_json::Value;

// Trait definition for all members referenced in the GuiElems HashMap
pub trait GuiElem {
    fn draw(&self, ui: &mut egui::Ui);
    fn update_data(&mut self, data: Value);
}
