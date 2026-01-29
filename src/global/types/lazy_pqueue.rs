use crate::global::types::{KeyHeap, KeyNode};
use std::{collections::HashSet, hash::Hash};

#[derive(Debug)]
pub struct LazyPQueue<T> {
    heap: KeyHeap<T>,
    lazy: HashSet<T>,
}

impl<T> LazyPQueue<T>
where
    T: Hash + Eq,
{
    pub fn new() -> Self {
        Self {
            heap: KeyHeap::new(),
            lazy: HashSet::new(),
        }
    }
    pub fn clear(&mut self) {
        self.heap.clear();
        self.lazy.clear();
    }
    pub fn push(&mut self, node: KeyNode<T>) {
        self.heap.push(node)
    }
    pub fn peek(&mut self) -> Option<&KeyNode<T>> {
        loop {
            let remove;
            if let Some(node) = self.heap.peek() {
                if self.lazy.contains(&node.coord) {
                    remove = true;
                } else {
                    return self.heap.peek();
                }
            } else {
                return None;
            }
            if remove {
                let node = self.heap.pop()?;
                self.lazy.remove(&node.coord);
            }
        }
    }
    pub fn remove(&mut self, coord: T) {
        self.lazy.insert(coord);
    }
    pub fn pop(&mut self) -> Option<KeyNode<T>> {
        while let Some(node) = self.heap.pop() {
            if self.lazy.contains(&node.coord) {
                self.lazy.remove(&node.coord);
            } else {
                return Some(node);
            }
        }
        None
    }
}
