#![allow(unused)]
use sabrina::environment::grid::Grid;
use sabrina::environment::info::reconstruct;
use sabrina::environment::quad::{QuadNode, QuadTree};
use sabrina::global::consts::{AXIS_MAX, LEVELS, PARTITION};
use sabrina::global::types::plan::{AStarPlan, BestFirstPlan, DStarPlan};
use sabrina::global::types::plan::{ForwardIter, PlanIter};
use sabrina::global::types::{Belief, Bounds, Coord, KeyNode, MinHeap, MinNode, SpatialMap};
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::read_grid;
use sabrina::parser::quad::read_quad;
use sabrina::sensor::lidar::Lidar;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::mem;
use std::time::Instant;

trait Planner<S: SpatialMap> {
    type Plan: PlanIter;
    fn plan(&self, env: &S, source: Coord, target: Coord) -> Option<Self::Plan>;
}

struct BestFirstPlanner<S: SpatialMap> {
    spatial: S,
}

impl<S: SpatialMap> Planner<S> for BestFirstPlanner<S> {
    type Plan = BestFirstPlan;
    fn plan(&self, env: &S, source: Coord, target: Coord) -> Option<Self::Plan> {
        if env.obstructed(target) {
            return None;
        };
        let mut p_queue: MinHeap = MinHeap::new();
        let mut enqueue: HashSet<Coord> = HashSet::new();
        let mut precursor = HashMap::new();
        p_queue.push(MinNode::new(0, source));
        enqueue.insert(source);
        let neighbors = [(1, 0), (0, 1), (!0, 0), (0, !0)];
        while let Some(node) = p_queue.pop() {
            if node.coord == target {
                let plan = reconstruct(&precursor, source, target);
                return Some(Self::Plan { plan: plan });
            }
            for (dx, dy) in neighbors {
                let n_xy = (node.coord.0.wrapping_add(dx), node.coord.1.wrapping_add(dy));
                if n_xy.0 < AXIS_MAX && n_xy.1 < AXIS_MAX {
                    if !enqueue.contains(&n_xy) && !env.obstructed(n_xy) {
                        precursor.insert(n_xy, node.coord);
                        enqueue.insert(n_xy);
                        let cost = env.distance(env.encode(n_xy), env.encode(target));
                        p_queue.push(MinNode::new(cost, n_xy));
                    }
                }
            }
        }
        None
    }
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
