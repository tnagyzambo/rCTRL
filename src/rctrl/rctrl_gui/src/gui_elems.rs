/// REFERENCE: <https://stackoverflow.com/questions/45786717/how-to-implement-hashmap-with-two-keys/45795699>
use std::collections::HashMap;
use std::hash::{Hash, Hasher};
use std::borrow::Borrow;

use crate::GuiElem;

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
// The value type Box<dyn GuiElem> can probably be moved to a generic value argument if this two key map ever has to be reused
// but for now this is not needed
pub struct GuiElems<A: Eq + Hash, B: Eq + Hash> {
    pub map: HashMap<(A, B), Box<dyn GuiElem>>,
}

// We must expose all the methods that we wish to call from the basic HashMap contained within GuiElems
impl<A: Eq + Hash, B: Eq + Hash> GuiElems<A, B> {
    pub fn new() -> Self {
        GuiElems {
            map: HashMap::new(),
        }
    }

    pub fn get_mut(&mut self, a: &A, b: &B) -> Option<&mut Box<dyn GuiElem>> {
        self.map.get_mut(&(a, b) as &dyn KeyPair<A, B>)
    }

    pub fn insert(&mut self, a: A, b: B, v: Box<dyn GuiElem>) {
        self.map.insert((a, b), v);
    }
}
