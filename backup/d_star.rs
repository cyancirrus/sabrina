use crate::global::types::LazyPQueue;
use crate::global::types::plan::Planner;
use crate::global::types::{Coord, DStarPlan, KeyNode, SpatialMap};
use std::collections::HashMap;
use std::hash::Hash;
use std::mem;

/// Distance Map for DStarLite : Coord -> (G, Rhs)
///
/// # Data Types
/// * Coord :: x, y coordinates
/// * (G, Rhs) ~ (usize, usize)
///
/// # Definitions
/// * G ~Independent current estimate of cost to go
/// * Rhs ~ Estimate of cost given neighbors belief

pub type Star<T> = HashMap<T, (G, Rhs)>;
/// Independent current estimate of cost to go
type G = usize;
// Estimate of cost given neighbors belief
type Rhs = usize;

pub struct DStarPlanner<S: SpatialMap> {
    star: Star<S::Encoded>,
    pqueue: LazyPQueue<S::Encoded>,
    k: usize,
}

impl<S: SpatialMap> DStarPlanner<S>
where
    // S::Encoded: Eq + Hash,
    S::Encoded: Eq + Hash + std::fmt::Debug + Eq,
{
    pub fn new() -> Self {
        Self {
            k: 0,
            star: Star::new(),
            pqueue: LazyPQueue::new(),
        }
    }
    fn update_vertex(&mut self, env: &S, coord: S::Encoded, source: S::Encoded) {
        if coord == source {
            return;
        }
        let mut min_distance = usize::MAX;
        for neigh in env.neighbors(coord) {
            let dh = env.distance(neigh, coord);
            if let Some(&(_g, rhs)) = self.star.get(&neigh) {
                min_distance = min_distance.min(rhs.saturating_add(dh));
            }
        }
        let h = env.distance(coord, source);
        match self.star.get_mut(&coord) {
            Some((g, rhs)) => {
                *rhs = min_distance;
                if g != rhs {
                    self.pqueue.push(KeyNode::new(coord, *g, *rhs, h, self.k));
                } else {
                    self.pqueue.remove(coord);
                }
            }
            None => {
                self.star.insert(coord, (min_distance, usize::MAX));
                self.pqueue
                    .push(KeyNode::new(coord, min_distance, min_distance, h, self.k));
            }
        }
    }
    fn improve_and_invalidate(&mut self, env: &S, source: S::Encoded) {
        let u = self.pqueue.pop().unwrap();
        let (g_u, rhs_u) = self.star.get_mut(&u.coord).unwrap();
        if g_u > rhs_u {
            // improvement
            *g_u = *rhs_u;
            self.pqueue.remove(u.coord);
            for p in env.neighbors(u.coord) {
                self.update_vertex(env, p, source);
            }
        } else {
            // invalidation
            *g_u = usize::MAX;
            self.update_vertex(env, u.coord, source);
        }
    }
    fn compute_shortest_path(&mut self, env: &S, source: S::Encoded, target: S::Encoded) {
        self.pqueue.push(KeyNode::new(target, 0, 0, 0, self.k));
        self.star.insert(target, (env.distance(source, target), 0));
        self.star
            .insert(source, (usize::MAX, env.distance(source, target)));
        // .insert(source, (env.distance(source, target), usize::MAX, ));

        loop {
            match (self.star.get(&source), self.pqueue.peek()) {
                (Some(&(g, rhs)), Some(top_key)) => {
                    let h = env.distance(source, target);
                    let start_key = KeyNode::new(source, g, rhs, h, self.k);
                    let k_old = top_key.cost_astar;

                    // inverted compare logic because min-heap reverses compare
                    if *top_key > start_key {
                        for p in env.neighbors(source) {
                            let node = KeyNode::new(
                                p,
                                usize::MAX,
                                rhs + env.distance(source, p),
                                env.distance(p, target),
                                self.k,
                            );
                            self.pqueue.push(node);
                        }
                    } else if g != rhs {
                        // println!("inconsistent");
                        // println!("---------------------");
                        // println!("source {source:?}");
                        // println!("---------------------");
                        self.improve_and_invalidate(env, target);
                    } else {
                        break;
                    }
                }
                _ => break,
            }
        }
    }
    fn reconstruct_decode(
        &mut self,
        env: &S,
        source: S::Encoded,
        target: S::Encoded,
    ) -> Option<Vec<Coord>> {
        println!("Star Appears as {:?}", self.star);
        println!("Working Reconstruction");
        // assert!(false);
        let mut plan = Vec::new();
        let mut node_curr = Some(source);
        let mut node_next;
        let mut current_cost;
        while let Some(current) = node_curr {
            current_cost = self.star[&current];
            println!("current {current:?}");
            if current != source {
                plan.push(env.decode(current));
            }
            if target == current {
                return Some(plan);
            }
            node_next = None;
            // let mut min_cost = usize::MAX;
            for neigh in env.neighbors(current) {
                if let Some(&cost) = self.star.get(&neigh) {
                    // let cost = g_n.saturating_add(env.distance(current, neigh));
                    if cost < current_cost {
                        current_cost = cost;
                        node_next = Some(neigh);
                    }
                }
            }
            mem::swap(&mut node_curr, &mut node_next);
        }
        None
    }
}

impl<S: SpatialMap> Planner<S> for DStarPlanner<S>
where
    S::Encoded: Eq + Hash + std::fmt::Debug + Eq,
{
    type Plan = DStarPlan;
    fn plan(&mut self, env: &S, source: Coord, target: Coord) -> Option<Self::Plan> {
        // self.pqueue.clear();
        println!("STARTED:: {source:?} -> {target:?}");
        if env.obstructed(target) {
            return None;
        };
        let (s_encode, t_encode) = (env.encode(source), env.encode(target));
        // encodes distance matrix
        self.compute_shortest_path(env, s_encode, t_encode);
        let decoded = self.reconstruct_decode(env, s_encode, t_encode);
        match decoded {
            Some(plan) => Some(Self::Plan { plan }),
            None => None,
        }
    }
    fn update(&mut self, _env: &S, _obstacle: Coord) {
        // fn update(&mut self, env: &S, obstacle:Coord) {
        // println!("putting block at {obstacle:?}");
        // let o_encode = env.encode(obstacle);
        // self.pqueue.push(KeyNode::new(o_encode, 0, usize::MAX, 0));
        // self.star.insert(o_encode, (usize::MAX, usize::MAX));
        // self.pqueue.remove(o_encode);
        // self.pqueue.push(KeyNode::new(o_encode, 0, usize::MAX, 0));
        // self.pqueue.push(KeyNode::new(o_encode, usize::MAX, 0, 0));
        // self.star.insert(o_encode, (usize::MAX, usize::MAX));
    }
}
