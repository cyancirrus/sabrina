use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::hash::Hash;

/// Priority Queue for DStarLite
///
/// # Node :: Arguments
/// * cost_star := min(g, rhs) + h;
/// * cost_dijsktra := min(g, rhs);
/// * coord := (x, y) cordinate
pub type KeyHeap<T> = BinaryHeap<KeyNode<T>>;

/// Represents the underlying DStarLite Estimates
///
/// # Arguments
/// * cost_star := min(g, rhs) + h;
/// * cost_dijsktra := min(g, rhs);
/// * coord := (x, y) cordinate
#[derive(Eq, PartialEq, Debug)]
pub struct KeyNode<T> {
    // All costs should be non negative
    pub cost_astar: usize,    // min(g, rhs) + h;
    pub cost_dijkstra: usize, // min(g, rhs)
    pub coord: T,
}

impl <T> KeyNode <T>
where T: Hash
{
    pub fn new(coord: T, g: usize, rhs: usize, h: usize) -> Self {
        let cost_dijkstra = g.min(rhs);
        let cost_astar = cost_dijkstra.saturating_add(h);
        Self {
            cost_astar,
            cost_dijkstra,
            coord,
        }
    }
}

impl <T> Ord for KeyNode <T>
where T: Hash + Eq
{
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost_astar
            .cmp(&self.cost_astar)
            .then_with(|| other.cost_dijkstra.cmp(&self.cost_dijkstra))
    }
}

impl <T> PartialOrd for KeyNode <T> 
where T: Hash + Eq
{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
