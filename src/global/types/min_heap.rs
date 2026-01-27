use crate::global::types::monolithic::Coord;
use std::cmp::Ordering;
#[derive(Eq, PartialEq, Debug)]
pub struct MinNode {
    // All costs should be non negative
    pub cost: usize,
    pub coord: Coord,
}

impl MinNode {
    pub fn new(cost: usize, coord: Coord) -> Self {
        Self { cost, coord }
    }
}

impl Ord for MinNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl PartialOrd for MinNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
