use crate::global::types::plan::BestFirstPlan;
use crate::global::types::plan::Planner;
use crate::global::types::{Belief, ACoord, MinHeap, MinNode, SpatialMap};
use std::collections::{HashMap, HashSet};

pub struct BestFirstPlanner;
impl BestFirstPlanner {
    fn encode_plan<S: SpatialMap>(
        &self,
        env: &S,
        source: S::Encoded,
        target: S::Encoded,
    ) -> Option<HashMap<S::Encoded, S::Encoded>>
// where S::Encoded: std::fmt::Debug + Eq + std::hash::Hash + Copy
    {
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
    ) -> Vec<ACoord> {
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
    fn plan(&mut self, env: &S, source: ACoord, target: ACoord) -> Option<Self::Plan> {
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
    fn update(&mut self, _: &S, _: ACoord, _: ACoord) {}
}
