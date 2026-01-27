use crate::global::types::monolithic::Coord;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// AStar Distance Based Priority Queue 
///
/// # Node :: Arguments
/// * coord := (x, y) cordinate
/// * cost := incurred_cost + heuristic(here, target);
/// * incurred := cost so far;
pub type HeurHeap = BinaryHeap<HeurMinNode>;

/// Heuristic Distance Based Priority
///
/// # Arguments
/// * coord := (x, y) cordinate
/// * cost := incurred_cost + heuristic(here, target);
/// * incurred := cost so far;
#[derive(Eq, PartialEq, Debug)]
pub struct HeurMinNode {
    // All costs should be non negative
    pub coord: Coord,
    pub cost: usize,
    pub incurred: usize,
}

impl HeurMinNode {
    pub fn new(cost: usize, coord: Coord) -> Self {
        Self {
            cost,
            coord,
            incurred: 0,
        }
    }
}

impl Ord for HeurMinNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl PartialOrd for HeurMinNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

