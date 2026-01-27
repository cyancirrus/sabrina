#![allow(unused)]
use sabrina::environment::grid::Grid;
use sabrina::environment::info::reconstruct;
use sabrina::environment::quad::{QuadNode, QuadTree};
use sabrina::global::consts::{LEVELS, PARTITION};
use sabrina::global::types::plan::{AStarPlan, DStarPlan, PlanIter};
use sabrina::global::types::{Belief, Bounds, Coord, KeyNode};
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::read_grid;
use sabrina::parser::quad::read_quad;
use sabrina::sensor::lidar::Lidar;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::mem;
use std::time::Instant;

// testing interface
trait SpatialMap {
    type Encoded: Copy;
    // sensor-facing (world space)
    fn insert_ray(&mut self, pos: Coord, hit: Coord, belief: Belief);
    // planner-facing (encoded space)
    fn get_belief(&self, node: Self::Encoded) -> Belief;
    fn neighbors(&self, node: Self::Encoded) -> Vec<Self::Encoded>;
    fn distance(&self, a: Self::Encoded, b: Self::Encoded) -> usize;
    // bridge
    fn encode(&self, coord: Coord) -> Self::Encoded;
    fn decode(&self, node: Self::Encoded) -> Coord;
}

fn main() {
    let x = vec![(1, 1), (2, 2), (3, 3)];
    let astar_plan = AStarPlan { plan: x.clone() };
    let x = vec![(3, 3), (2, 2), (1, 1)];
    let dstar_plan = DStarPlan { plan: x.clone() };
    assert!(
        astar_plan
            .iter()
            .zip(dstar_plan.iter())
            .all(|(a, d)| a == d),
    );
}
