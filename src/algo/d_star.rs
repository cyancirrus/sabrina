#![allow(dead_code, unused_variables)]
// use crate::global::types::LazyPQueue;
use crate::global::types::IPQueue;
use crate::global::types::plan::Planner;
use crate::global::types::{ACoord, DStarPlan, SpatialMap, StarKey};
use std::collections::HashMap;
use std::hash::Hash;
use std::mem;

/// Distance Map for DStarLite : ACoord -> (G, Rhs)
///
/// # Data Types
/// * ACoord :: x, y coordinates
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
    pqueue: IPQueue<StarKey, S::Encoded>,
    source: Option<S::Encoded>,
    target: Option<S::Encoded>,
    k: usize,
}

impl<S: SpatialMap> DStarPlanner<S>
where
    S::Encoded: Eq + Hash + std::fmt::Debug + Eq,
{
    pub fn new() -> Self {
        Self {
            star: Star::new(),
            pqueue: IPQueue::new(),
            source: None,
            target: None,
            k: 0,
        }
    }
    fn initialize(&mut self, env: &S, source: S::Encoded, target: S::Encoded) {
        self.star.clear();
        self.pqueue.clear();
        self.k = 0;
        self.source = Some(source);
        self.target = Some(target);
        let h = env.distance(source, target);
        self.star.insert(target, (usize::MAX, 0));
        self.star.insert(source, (usize::MAX, usize::MAX));
        println!("default {:?}", self.star);
        self.pqueue.push(
            target,
            StarKey {
                cost_astar: h,
                cost_dijkstra: 0,
            },
        );
    }
    fn calculate_key(&self, env: &S, u: S::Encoded) -> StarKey {
        let source = self.source.unwrap();
        let target = self.target.unwrap();
        let h = env.distance(u, source);
        let &(g, rhs) = self.star.get(&u).unwrap_or(&(usize::MAX, usize::MAX));
        StarKey::new(g, rhs, h, self.k)
    }
    fn update_vertex(&mut self, env: &S, u: S::Encoded) {
        let (g, rhs) = self.star.get(&u).unwrap_or(&(usize::MAX, usize::MAX));
        if g != rhs {
            let ckey = self.calculate_key(env, u);
            self.pqueue.push(u, ckey);
        } else {
            self.pqueue.remove(&u);
        }
    }
    fn propagate_cost_rhs(&mut self, env: &S, u: S::Encoded) {
        let target = self.target.unwrap();
        let (g_u, _) = self.star[&u];
        for s in env.neighbors(u) {
            if s == target {
                continue;
            }
            let rhs_new = env.distance(s, u).saturating_add(g_u);
            let (g, rhs) = self.star.entry(s).or_insert((usize::MAX, usize::MAX));
            let rhs_updated = (*rhs).min(rhs_new);
            if rhs_new < *rhs {
                *rhs = rhs_new;
            }
            self.update_vertex(env, s);
        }
    }
    fn find_min_neighbor_g(&self, env: &S, s: S::Encoded) -> usize {
        let mut min_cost = usize::MAX;
        for s_p in env.neighbors(s) {
            if let Some(&(g_sp, _)) = self.star.get(&s_p) {
                min_cost = min_cost.min(env.distance(s, s_p).saturating_add(g_sp));
            }
        }
        min_cost
    }
    fn propagate_cost_g(&mut self, env: &S, u: S::Encoded, g_old: usize) {
        let target = self.target.unwrap();
        if u == target {
            return;
        }
        for s in env.neighbors(u) {
            // only update if not equal

            let &(g_s, rhs) = self.star.get(&s).unwrap_or(&(usize::MAX, usize::MAX));
            if rhs != env.distance(u, s).saturating_add(g_old) {
                continue;
            }
            let rhs_new = self.find_min_neighbor_g(env, s);
            self.star.insert(s, (g_s, rhs_new));
            self.update_vertex(env, s);
        }
    }
    fn compute_shortest_path(&mut self, env: &S) {
        println!("compute");
        let source = self.source.unwrap();
        let target = self.target.unwrap();
        loop {
            let (g, rhs) = self.star[&source];
            if let Some((top_coord, top_key)) = self.pqueue.peek() {
                let start_key = self.calculate_key(env, source);
                if g == rhs && top_key <= start_key {
                    break;
                }
            }
            let (u_coord, k_old) = self.pqueue.pop().unwrap();
            let &(g_u, rhs_u) = match self.star.get(&u_coord) {
                Some(entry) => entry,
                None => continue,
            };
            let k_new = self.calculate_key(env, u_coord);
            // reversed due to starkey reversed compare for order for minheap
            if k_old > k_new {
                self.pqueue.push(u_coord, k_new);
            } else if g_u > rhs_u {
                self.star.insert(u_coord, (rhs_u, rhs_u));
                self.pqueue.remove(&u_coord);
                self.propagate_cost_rhs(env, u_coord);
            } else {
                let g_old = g_u;
                self.star.insert(u_coord, (usize::MAX, rhs_u));
                self.propagate_cost_g(env, u_coord, g_old);
                self.update_vertex(env, u_coord);
            }
        }
    }
    fn reconstruct_decode(&mut self, env: &S) -> Option<Vec<ACoord>> {
        println!("reconstruct");
        let source = self.source.unwrap();
        let target = self.target.unwrap();
        let mut plan = Vec::new();
        let mut node_curr = Some(source);
        let mut node_next;
        let mut best_cost;
        while let Some(current) = node_curr {
            println!("current {current:?}");
            if current != source {
                plan.push(env.decode(current));
            }
            if target == current {
                return Some(plan);
            }
            node_next = None;
            best_cost = usize::MAX;
            for neigh in env.neighbors(current) {
                if let Some(&(g_n, rhs)) = self.star.get(&neigh) {
                    let cost = g_n.saturating_add(env.distance(current, neigh));
                    if cost < best_cost {
                        best_cost = cost;
                        node_next = Some(neigh);
                    }
                }
            }
            mem::swap(&mut node_curr, &mut node_next);
        }
        None
    }

    fn new_plan(&mut self, env: &S, source: ACoord, target: ACoord) {
        let s_encode = env.encode(source);
        let t_encode = env.encode(target);
        self.source = Some(s_encode);
        self.target = Some(t_encode);
        if env.obstructed(target) {
            return;
        };
        // encodes distance matrix
        self.initialize(env, s_encode, t_encode);
        self.compute_shortest_path(env);
    }
    fn revise_plan(&mut self, env: &S) {
        let source = self.source.unwrap();
        let (g, rhs) = self.star[&source];
        self.update_vertex(env, source);
        self.compute_shortest_path(env);
    }
    fn revise_bounds(&mut self, env: &S, source: ACoord, target: ACoord) {
        let s_old = self.source.unwrap();
        let t_old = self.target.unwrap();
        let s_new = env.encode(source);
        let t_new = env.encode(target);
        if s_new != s_old {
            let (g_old, rhs_old) = self.star[&s_old];
            self.k += env.distance(s_old, s_new);
            self.star.remove(&s_old);
            self.propagate_cost_g(env, s_old, g_old);
            self.propagate_cost_g(env, s_new, g_old);
            self.update_vertex(env, s_old);
            self.update_vertex(env, s_new);
            self.source = Some(s_new);
            self.star.insert(s_new, (g_old, rhs_old));
        }
        if t_new != t_old {
            let (g_old, rhs_old) = self.star[&t_old];
            self.star.remove(&t_old);
            self.star.insert(t_new, (g_old, rhs_old));
            // self.propagate_cost_g(env, t_old, g_old);
            self.propagate_cost_g(env, t_new, g_old);
            // self.update_vertex(env, t_old);
            self.update_vertex(env, t_new);
            self.target = Some(t_new);
            let h = env.distance(s_old, t_new);
            self.pqueue.push(
                t_new,
                StarKey {
                    cost_astar: h,
                    cost_dijkstra: 0,
                },
            );
        }
    }
}

impl<S: SpatialMap> Planner<S> for DStarPlanner<S>
where
    S::Encoded: Eq + Hash + std::fmt::Debug + Eq,
{
    type Plan = DStarPlan;
    fn plan(&mut self, env: &S, source: ACoord, target: ACoord) -> Option<Self::Plan> {
        if self.source.is_none() || self.target.is_none() {
            println!("star appear as \n {:?}", self.star);
            self.new_plan(env, source, target);
        } else {
            self.revise_bounds(env, source, target);
            println!("star appear as \n {:?}", self.star);
            self.revise_plan(env);
        }
        println!("end star appear as \n {:?}", self.star);
        match self.reconstruct_decode(env) {
            Some(mut plan) => {
                plan.push(target);
                Some(Self::Plan { plan })
            }
            None => None,
        }
    }
    fn update(&mut self, env: &S, position: ACoord, obstacle: ACoord) {
        // TODO: Need to integrate rhs for when we find better paths
        if self.source.is_none() || self.target.is_none() {
            return;
        }
        let node = env.encode(obstacle);
        // if worse
        let &(g_obs, rhs_obs) = self.star.get(&node).unwrap_or(&(usize::MAX, usize::MAX));
        self.star.remove(&node);
        self.propagate_cost_g(env, node, g_obs);
        self.update_vertex(env, node);
    }
}
