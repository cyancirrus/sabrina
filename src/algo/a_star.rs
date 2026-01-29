use crate::global::types::plan::AStarPlan;
use crate::global::types::plan::Planner;
use crate::global::types::{Belief, Coord, HeurHeap, HeurNode, SpatialMap};
use std::collections::{HashMap, HashSet};

pub struct AStarPlanner;

impl AStarPlanner {
    fn encode_plan<S: SpatialMap>(
        &self,
        env: &S,
        source: S::Encoded,
        target: S::Encoded,
    ) -> Option<HashMap<S::Encoded, S::Encoded>> {
        let mut p_queue: HeurHeap<S::Encoded> = HeurHeap::new();
        let mut enqueue: HashSet<S::Encoded> = HashSet::new();
        let mut precursor = HashMap::new();
        p_queue.push(HeurNode {
            incurred: 0,
            cost: env.distance(source, target),
            coord: source,
        });
        enqueue.insert(source);
        while let Some(node) = p_queue.pop() {
            if node.coord == target {
                return Some(precursor);
            }
            for n_xy in env.neighbors(node.coord) {
                if enqueue.insert(n_xy) && env.belief(n_xy) != Belief::Occupied {
                    precursor.insert(n_xy, node.coord);
                    let heuristic = env.distance(n_xy, target);
                    let incurred = node.incurred + env.distance(node.coord, n_xy);
                    p_queue.push(HeurNode {
                        incurred,
                        cost: incurred + heuristic,
                        coord: n_xy,
                    });
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

impl<S: SpatialMap> Planner<S> for AStarPlanner {
    type Plan = AStarPlan;
    fn plan(&mut self, env: &S, source: Coord, target: Coord) -> Option<Self::Plan> {
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
    fn update(&mut self, _: &S, _: Coord) {}
}
