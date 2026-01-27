#![allow(unused)]
use sabrina::environment::grid::Grid;
use sabrina::environment::info::reconstruct;
use sabrina::environment::quad::{QuadNode, QuadTree};
use sabrina::global::consts::{AXIS_MAX, LEVELS, PARTITION};
use sabrina::global::types::plan::{AStarPlan, BestFirstPlan, DStarPlan};
use sabrina::global::types::plan::{ForwardIter, PlanIter};
use sabrina::global::types::{Belief, Bounds, Coord, KeyNode, MinHeap, MinNode, SpatialMap};
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::sensor::lidar::Lidar;
use std::collections::{BinaryHeap, HashMap, HashSet};

trait Planner<S: SpatialMap> {
    type Plan: PlanIter;
    fn plan(&self, env: &S, source: Coord, target: Coord) -> Option<Self::Plan>;
}

struct BestFirstPlanner;

impl BestFirstPlanner {
    fn encode_plan<S: SpatialMap>(
        &self,
        env: &S,
        source: S::Encoded,
        target: S::Encoded,
    ) -> Option<HashMap<S::Encoded, S::Encoded>> {
        let mut p_queue: MinHeap<S::Encoded> = MinHeap::new();
        let mut enqueue: HashSet<S::Encoded> = HashSet::new();
        let mut precursor = HashMap::new();
        p_queue.push(MinNode::new(0, source));
        enqueue.insert(source);
        while let Some(node) = p_queue.pop() {
            if node.coord == target {
                return Some(precursor);
            }
            for n_xy in env.neighbors(node.coord) {
                if enqueue.insert(n_xy) && env.belief(n_xy) != Belief::Occupied {
                    precursor.insert(n_xy, node.coord);
                    let cost = env.distance(n_xy, target);
                    p_queue.push(MinNode::new(cost, n_xy));
                }
            }
        }
        None
    }
    pub fn reconstruct_decode<S: SpatialMap>(
        &self,
        env: &S,
        precursor: &HashMap<S::Encoded, S::Encoded>,
        source: S::Encoded,
        target: S::Encoded,
    ) -> Vec<Coord> {
        // Ensure this is synchronized with action as this returns reversed plan
        let mut plan = vec![];
        let mut node = target;
        while node != source {
            plan.push(env.decode(node));
            node = precursor[&node];
        }
        plan
    }
}

impl<S: SpatialMap> Planner<S> for BestFirstPlanner {
    type Plan = BestFirstPlan;
    fn plan(&self, env: &S, source: Coord, target: Coord) -> Option<Self::Plan> {
        if env.obstructed(target) {
            return None;
        };
        let (s_encode, t_encode) = (env.encode(source), env.encode(target));

        let precursor = self.encode_plan(env, s_encode, t_encode);
        match precursor {
            Some(map) => {
                let plan = self.reconstruct_decode(env, &map, s_encode, t_encode);
                Some(Self::Plan { plan })
            }
            None => None,
        }
    }
}
// fn plan(&self, env: &S, source: Coord, target: Coord) -> Option<Self::Plan> {
//     if env.obstructed(target) {
//         return None;
//     };
//     let (s_encode, t_encode) = (env.encode(source), env.encode(target));
//     let mut p_queue: MinHeap<S::Encoded> = MinHeap::new();
//     let mut enqueue: HashSet<S::Encoded> = HashSet::new();
//     let mut precursor = HashMap::new();
//     p_queue.push(MinNode::new(0, s_encode));
//     enqueue.insert(s_encode);
//     while let Some(node) = p_queue.pop() {
//         if node.coord == t_encode {
//             let plan = reconstruct(&precursor, source, target);
//             return Some(Self::Plan { plan: plan });
//         }
//         for n_xy_encoded in env.neighbors(node.coord) {
//             if enqueue.insert(n_xy_encoded) && env.belief(n_xy_encoded) != Belief::Occupied {
//                 let n_xy = env.decode(n_xy_encoded);
//                 let p_xy = env.decode(node.coord);
//                 precursor.insert(n_xy, p_xy);
//                 let cost = env.distance(n_xy_encoded, t_encode);
//                 p_queue.push(MinNode::new(cost, n_xy_encoded));
//             }
//         }
//     }
//     None
// }
// }

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
