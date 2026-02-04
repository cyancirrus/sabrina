use crate::global::types::{ACoord, Belief};
use std::hash::Hash;

pub trait SpatialMap {
    type Encoded: Copy + Eq + PartialEq + Hash;
    // // sensor-facing (world space)
    fn insert_ray(&mut self, pos: ACoord, hit: ACoord);
    fn obstructed(&self, coord: ACoord) -> bool;
    // // planner-facing (encoded space)
    fn belief(&self, node: Self::Encoded) -> Belief;
    fn neighbors(&self, node: Self::Encoded) -> Vec<Self::Encoded>;
    fn distance(&self, a: Self::Encoded, b: Self::Encoded) -> usize;
    // bridge
    fn encode(&self, coord: ACoord) -> Self::Encoded;
    fn decode(&self, node: Self::Encoded) -> ACoord;
}
