use crate::global::types::{Belief, Coord};

pub trait SpatialMap {
    type Encoded: Copy;
    // // sensor-facing (world space)
    fn insert_ray(&mut self, pos: Coord, hit: Coord);
    fn obstructed(&self, coord: Coord) -> bool;
    // // planner-facing (encoded space)
    fn belief(&self, node: Self::Encoded) -> Belief;
    fn neighbors(&self, node: Self::Encoded) -> Vec<Self::Encoded>;
    fn distance(&self, a: Self::Encoded, b: Self::Encoded) -> usize;
    // bridge
    fn encode(&self, coord: Coord) -> Self::Encoded;
    fn decode(&self, node: Self::Encoded) -> Coord;
}
