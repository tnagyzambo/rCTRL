use eframe::egui;
use serde_json::Value;
use std::borrow::Borrow;
use std::collections::HashMap;
use std::hash::{Hash, Hasher};
use std::rc::Rc;
use std::sync::Mutex;

// All GuiElem's must have a unique id
// Simple global counter
lazy_static! {
    static ref GUI_ELEM_COUNTER: Mutex<u32> = Mutex::new(0);
}

// Fucntion to access and increment the global counter
pub fn gen_gui_elem_id() -> u32 {
    let mut id_counter = GUI_ELEM_COUNTER.lock().unwrap();
    *id_counter += 1;
    return *id_counter;
}

// Trait definition for all members referenced in the GuiElems HashMap
pub trait GuiElem {
    fn draw(&mut self, ui: &mut egui::Ui);
    fn update_data(&mut self, data: &Value);
    fn deregister(&self);
}

#[derive(Eq, PartialEq, Hash)]
struct A(&'static str);

#[derive(Eq, PartialEq, Hash)]
struct B(&'static str);

trait KeyPair<A, B> {
    fn a(&self) -> &A;
    fn b(&self) -> &B;
}

impl<A, B> KeyPair<A, B> for (A, B) {
    fn a(&self) -> &A {
        &self.0
    }
    fn b(&self) -> &B {
        &self.1
    }
}

impl<A, B> KeyPair<A, B> for (&A, &B) {
    fn a(&self) -> &A {
        self.0
    }
    fn b(&self) -> &B {
        self.1
    }
}

// Most HashMap methods require that the key implement the Borrow<Q> trait
// We must implement Borrow<Q> for our key pair
// REFERENCE: <https://doc.rust-lang.org/std/collections/struct.HashMap.html#method.get_mut>
impl<'a, A, B> Borrow<dyn KeyPair<A, B> + 'a> for (A, B)
where
    A: Eq + Hash + 'a,
    B: Eq + Hash + 'a,
{
    fn borrow(&self) -> &(dyn KeyPair<A, B> + 'a) {
        self
    }
}

// Similarly the Hash trait must also be implemented
impl<A: Hash, B: Hash> Hash for dyn KeyPair<A, B> + '_ {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.a().hash(state);
        self.b().hash(state);
    }
}

// Similarly the PartialEq trait must also be implemented
impl<A: Eq, B: Eq> PartialEq for dyn KeyPair<A, B> + '_ {
    fn eq(&self, other: &Self) -> bool {
        self.a() == other.a() && self.b() == other.b()
    }
}

// Similarly the Eq trait must also be implemented
impl<A: Eq, B: Eq> Eq for dyn KeyPair<A, B> + '_ {}

// GuiElems can now be defined as an abstraction of a HashMap that takes a generic key pair
// The indiviual keys must implement the Eq and Hash traits
// A third key "ID" is used to index the internal hash map that allow multiple unique GuiElems to reference the same node/topic key pair
// This ID must implement copy to avoid some of the strange cases of leaking a mutable reference to the inner map from the shared outer map
// The value type HashMap<C, Rc<Mutex<dyn GuiElem>>> can probably be moved to a generic value argument if this key map ever has to be reused
// but for now this is not needed
/// REFERENCE: <https://stackoverflow.com/questions/45786717/how-to-implement-hashmap-with-two-keys/45795699>
pub struct GuiElems<A: Eq + Hash, B: Eq + Hash, C: Eq + Hash + Copy> {
    pub map: HashMap<(A, B), HashMap<C, Rc<Mutex<dyn GuiElem>>>>,
}

impl<A: Eq + Hash, B: Eq + Hash, C: Eq + Hash + Copy> Default for GuiElems<A, B, C> {
    fn default() -> Self {
        Self { map: HashMap::default() }
    }
}

// We must expose all the methods that we wish to call from the basic HashMap contained within GuiElems
impl<A: Eq + Hash, B: Eq + Hash, C: Eq + Hash + Copy> GuiElems<A, B, C> {
    pub fn new() -> Self {
        GuiElems { map: HashMap::new() }
    }

    // There is some issue with dereferencing the Rc<RwLock<dyn GuiElem>> when implementing get() as described in the reference material
    // The cleanest way that I've for getting an imutable reference to the underlying value is to clone the Rc<> that wraps everything
    // This makes sense as we are increasing the reference cound to the underlying RwLock<dyn GuiElem>> by one when an match is made
    pub fn get(&self, a: &A, b: &B, id: &C) -> Option<Rc<Mutex<dyn GuiElem>>> {
        match self.map.get(&(a, b) as &dyn KeyPair<A, B>) {
            Some(inner_map) => match inner_map.get(id) {
                Some(value) => Some(Rc::clone(&value)),
                None => None,
            },
            None => None,
        }
    }

    pub fn get_ids(&self, a: &A, b: &B) -> Option<Vec<C>> {
        match self.map.get(&(a, b) as &dyn KeyPair<A, B>) {
            Some(inner_map) => {
                let keys: Vec<C> = inner_map.keys().map(|&x| x).collect();
                Some(keys)
            }
            None => None,
        }
    }

    // Provides a mutable reference the the Rc<> that wraps the underlying data, allowing us to point to different data
    pub fn get_mut(&mut self, a: &A, b: &B, id: &C) -> Option<&mut Rc<Mutex<dyn GuiElem>>> {
        match self.map.get_mut(&(a, b) as &dyn KeyPair<A, B>) {
            Some(inner_map) => match inner_map.get_mut(id) {
                Some(value) => Some(value),
                None => None,
            },
            None => None,
        }
    }

    pub fn insert(&mut self, a: A, b: B, id: C, v: Rc<Mutex<dyn GuiElem>>) {
        match self.map.get_mut(&(&a, &b) as &dyn KeyPair<A, B>) {
            Some(inner_map) => {
                inner_map.insert(id, v);
                return;
            }
            None => {
                let mut inner_map = HashMap::<C, Rc<Mutex<dyn GuiElem>>>::new();
                inner_map.insert(id, v);
                self.map.insert((a, b), inner_map);
                return;
            }
        }
    }

    pub fn remove(&mut self, a: A, b: B, id: C) {
        match self.map.get_mut(&(&a, &b) as &dyn KeyPair<A, B>) {
            Some(inner_map) => {
                inner_map.remove(&id);
                return;
            }
            None => (),
        }
    }
}
