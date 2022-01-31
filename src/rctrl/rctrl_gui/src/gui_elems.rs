use std::collections::HashMap;
use std::hash::{Hash, Hasher};
use std::borrow::Borrow;

use crate::GuiElem;

trait KeyPair<A, B> {
    fn a(&self) -> &A;
    fn b(&self) -> &B;
}

impl<'a, A, B> Borrow<dyn KeyPair<A, B> + 'a> for (A, B)
where
    A: Eq + Hash + 'a,
    B: Eq + Hash + 'a,
{
    fn borrow(&self) -> &(dyn KeyPair<A, B> + 'a) {
        self
    }
}

impl<A: Hash, B: Hash> Hash for dyn KeyPair<A, B> + '_ {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.a().hash(state);
        self.b().hash(state);
    }
}

impl<A: Eq, B: Eq> PartialEq for dyn KeyPair<A, B> + '_ {
    fn eq(&self, other: &Self) -> bool {
        self.a() == other.a() && self.b() == other.b()
    }
}

impl<A: Eq, B: Eq> Eq for dyn KeyPair<A, B> + '_ {}

pub struct GuiElems<A: Eq + Hash, B: Eq + Hash> {
    pub map: HashMap<(A, B), Box<dyn GuiElem>>,
}

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

#[derive(Eq, PartialEq, Hash)]
struct A(&'static str);

#[derive(Eq, PartialEq, Hash)]
struct B(&'static str);
