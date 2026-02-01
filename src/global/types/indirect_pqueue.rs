use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::hash::Hash;

// Target DStarPlan { plan: [(1, 2), (1, 3), (2, 3), (3, 3), (4, 3), (5, 3), (6, 3), (7, 3), (8, 3), (9, 3), (10, 3), (11, 3), (12, 3), (13, 3), (14, 3), (15, 3), (16, 3), (17, 3), (18, 3)] }

// TODO: I think perhaps, the lazypqueue needs to be indirect pqueue, where the queue priority is
// based upon the current belief in priority, but the data is in reference to the propogated belief
// or known new state, ie... minheap < cost, node_id> <=> hashmap<node_id, (g,rhs)>

/// Indirect Priority Queue
///
/// # Updates queue based on new information

type IndirectHeap<M, S> = BinaryHeap<IndexNode<M, S>>;
pub struct IPQueue<M, S> {
    // priority -> key
    index: IndirectHeap<M, S>,
    // key -> measure
    map: HashMap<S, M>,
}

#[derive(Eq, PartialEq)]
pub struct IndexNode<M, S> {
    pub measure: M,
    // coord
    pub identity: S,
}

impl<M, S> IndexNode<M, S>
where
    S: Eq + PartialEq,
{
    pub fn new(measure: M, identity: S) -> Self {
        Self { measure, identity }
    }
}

impl<M, S> Ord for IndexNode<M, S>
where
    M: Eq + PartialEq + Ord + PartialOrd,
    S: Eq + PartialEq,
{
    fn cmp(&self, other: &Self) -> Ordering {
        other.measure.cmp(&self.measure)
    }
}

impl<M, S> PartialOrd for IndexNode<M, S>
where
    M: Eq + PartialEq + Ord + PartialOrd,
    S: Eq + PartialEq,
{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<M, S> IPQueue<M, S>
where
    M: Ord + PartialOrd + Copy,
    S: Eq + Copy + Hash,
{
    pub fn new() -> Self {
        Self {
            index: IndirectHeap::new(),
            map: HashMap::new(),
        }
    }
    pub fn clear(&mut self) {
        self.index.clear();
        self.map.clear();
    }
    pub fn peek(&mut self) -> Option<(S, M)> {
        loop {
            let k = self.index.peek()?;
            if let Some(v) = self.map.get(&k.identity) {
                if k.measure == *v {
                    return Some((k.identity, *v));
                }
            }
            // heap has removed the element alligning index with map
            self.index.pop();
        }
    }
    pub fn pop(&mut self) -> Option<(S, M)> {
        while let Some(k) = self.index.pop() {
            if !self.map.contains_key(&k.identity) {
                continue;
            }
            if k.measure == self.map[&k.identity] {
                let v = self.map.remove(&k.identity)?;
                return Some((k.identity, v));
            }
        }
        None
    }
    pub fn push(&mut self, identity: S, measure: M) {
        self.index.push(IndexNode { identity, measure });
        self.map.insert(identity, measure);
    }
    pub fn remove(&mut self, identity: &S) {
        self.map.remove(&identity);
    }
}

