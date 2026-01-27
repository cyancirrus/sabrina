use crate::global::types::monolithic::Coord;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

pub type KeyHeap = BinaryHeap<KeyNode>;

/// Represents the underlying DStarLite Estimates
///
/// # Arguments
/// * cost_star := min(g, rhs) + h;
/// * cost_dijsktra := min(g, rhs);
/// * coord := (x, y) cordinate
#[derive(Eq, PartialEq, Debug)]
pub struct KeyNode {
    // All costs should be non negative
    pub cost_astar: usize,    // min(g, rhs) + h;
    pub cost_dijkstra: usize, // min(g, rhs)
    pub coord: Coord,
}

impl KeyNode {
    pub fn new(coord: Coord, g: usize, rhs: usize, h: usize) -> Self {
        let cost_dijkstra = g.min(rhs);
        let cost_astar = cost_dijkstra.saturating_add(h);
        Self {
            cost_astar,
            cost_dijkstra,
            coord,
        }
    }
}

impl Ord for KeyNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost_astar
            .cmp(&self.cost_astar)
            .then_with(|| other.cost_dijkstra.cmp(&self.cost_dijkstra))
    }
}

impl PartialOrd for KeyNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
