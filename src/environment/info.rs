use crate::global::types::Coord;
use std::collections::HashMap;

pub fn reconstruct(
    precursor: &HashMap<Coord, Coord>,
    source: Coord,
    target: Coord,
) -> Vec<Coord> {
    // Ensure this is synchronized with action as this returns reversed plan
    let mut plan = vec![];
    let mut node = target;
    while node != source {
        plan.push(node);
        node = precursor[&node];
    }
    plan
}
