use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// AStar Distance Based Priority Queue
///
/// # Node :: Arguments
/// * coord := (x, y) cordinate
/// * cost := incurred_cost + heuristic(here, target);
/// * incurred := cost so far;
pub type HeurHeap<T> = BinaryHeap<HeurNode<T>>;

/// Heuristic Distance Based Priority
///
/// # Arguments
/// * coord := (x, y) cordinate
/// * cost := incurred_cost + heuristic(here, target);
/// * incurred := cost so far;
#[derive(Eq, PartialEq, Debug)]
pub struct HeurNode<T> {
    // All costs should be non negative
    pub cost: usize,
    pub incurred: usize,
    pub coord: T,
}

impl<T> Ord for HeurNode<T>
where
    T: Eq + PartialEq,
{
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl<T> PartialOrd for HeurNode<T>
where
    T: Eq + PartialEq,
{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
