#![allow(dead_code, unused_variables)]
// use crate::global::types::LazyPQueue;
use crate::global::types::IPQueue;
use crate::global::types::plan::Planner;
use crate::global::types::{Coord, DStarPlan, SpatialMap, StarKey};
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
    pqueue: IPQueue<StarKey, S::Encoded>,
    origin: Option<S::Encoded>,
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
            origin: None,
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
        self.source = Some(source);
        self.target = Some(target);
        let h = env.distance(source, target);
        self.star.insert(target, (usize::MAX, 0));
        self.star.insert(source, (usize::MAX, usize::MAX));
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
        StarKey::new(g, rhs, h, self.k)  // key uses min(g, rhs) as first component
    }
    fn update_vertex(&mut self, env: &S, u: S::Encoded) {
        let source = self.source.unwrap();
        let (g, rhs) = self.star.get(&u).unwrap_or(&(usize::MAX, usize::MAX));
        if g != rhs {
            let ckey = self.calculate_key(env, u);
            // insert and update
            self.pqueue.push(u, ckey);
        } else {
            // remove
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
            if u == self.source.unwrap() {
                println!("updating in propogate_cost_rhs {u:?}");
                println!("new value {rhs_new:?}");

            }
            }
            self.update_vertex(env, s);
        }
    }
    fn find_min_neighbor_g(&self, env: &S, s: S::Encoded) -> usize {
        let target = self.target.unwrap();
        let mut min_cost = usize::MAX;
        for n in env.neighbors(s) {
            if let Some(&(g, _)) = self.star.get(&n) {
                min_cost = min_cost.min(env.distance(s, n).saturating_add(g));
            }
        }
        min_cost
    }
    fn propagate_g_invalidation(&mut self, env: &S, u: S::Encoded, g_old:usize) {
        let source = self.source.unwrap();
        let target = self.target.unwrap();
        if u == target || u == source{
            return;
        }
        for n in env.neighbors(u) {
            if n == target || n == source {
                continue;
            }
            // only update if not equal
            let g = if let Some(&(g_s, rhs)) = self.star.get(&n) {
                if rhs != env.distance(u, n).saturating_add(g_old) {
                    continue;
                }
                g_s
            } else {
                continue;
            };
            let rhs_new = self.find_min_neighbor_g(env, n);
            self.star.remove(&n);
            // self.star.insert(n, (usize::MAX, rhs_new));
            // self.star.insert(n, (usize::MAX, usize::MAX));
            if n == self.source.unwrap() {
                println!("updating in propogateg_invalidation {u:?}");
                println!("new value {rhs_new:?}");

            }
        }
    }
    fn propagate_cost_g(&mut self, env: &S, u: S::Encoded, g_old: usize) {
        let target = self.target.unwrap();
        if u == target {
            return;
        }
        for n in env.neighbors(u) {
            // only update if not equal
            let g = if let Some(&(g_s, rhs)) = self.star.get(&n) {
                if rhs != env.distance(u, n).saturating_add(g_old) {
                    continue;
                }
                g_s
            } else {
                continue;
            };
            let rhs_new = self.find_min_neighbor_g(env, n);
            self.star.insert(n, (g, rhs_new));
        }
    }
    fn compute_shortest_path(&mut self, env: &S) {
        println!("-------------------------------------------");
        println!("in compute");
        println!("star {:?}", self.star);
        println!("-------------------------------------------");
        let source = self.source.unwrap();
        let target = self.target.unwrap();
        loop {
            let (g, rhs) = self.star[&source];
            if let Some((top_coord, top_key)) = self.pqueue.peek() {
                let start_key = self.calculate_key(env, source);
                if g == rhs && top_key <= start_key {
                    println!("top_key {top_key:?}, top_coord {top_coord:?}");
                    println!("start_key {start_key:?}");
                    break;
                }
            } else {
                break;
            }
            let (u_coord, k_old) = self.pqueue.pop().unwrap();
            println!("processing {u_coord:?}");
            let &(g_u, rhs_u) = match self.star.get(&u_coord) {
                Some(entry) => entry,
                None => continue,
            };
            let k_new = self.calculate_key(env, u_coord);
            // reversed due to starkey reversed compare for order for minheap
            if k_old > k_new {
                println!("u_coord {u_coord:?}");
                println!("k_old {k_old:?}");
                println!("k_new {k_new:?}");
                self.pqueue.push(u_coord, k_new);
            } else if g_u > rhs_u {
                // println!("-------------");
                // println!("g > rhs");
                if u_coord == self.source.unwrap() {
                    println!("updating in compute_shortest_path {u_coord:?}");
                    println!("new g_value {rhs_u:?}");

                }
                self.star.insert(u_coord, (rhs_u, rhs_u));
                self.pqueue.remove(&u_coord);
                self.propagate_cost_rhs(env, u_coord);
                self.update_vertex(env, u_coord);
            } else {
                println!("over here");
                let g_old = g_u;
                self.star.insert(u_coord, (usize::MAX, rhs_u));
                self.propagate_cost_g(env, u_coord, g_old);
                self.update_vertex(env, u_coord);
            }
        }
    }
    // (20, 19), (21, 19)
    fn reconstruct_decode(&mut self, env: &S) -> Option<Vec<Coord>> {
        println!("-------------------------------------------");
        println!("in decode");
        println!("star {:?}", self.star);
        println!("-------------------------------------------");
        let source = self.source.unwrap();
        let target = self.target.unwrap();
        let mut plan = Vec::new();
        let mut node_curr = Some(source);
        let mut node_next;
        let mut best_cost;
        while let Some(current) = node_curr {
            // println!("current {current:?}");
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

    fn new_plan(&mut self, env: &S, source: Coord, target: Coord) {
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
        // let (g, rhs) = self.star.get_mut(&source).unwrap();
        // *g = usize::MAX;
        // self.star.entry(source).or_insert((usize::MAX, usize::MAX));
        self.update_vertex(env, source);
        self.compute_shortest_path(env);
    }
}

impl<S: SpatialMap> Planner<S> for DStarPlanner<S>
where
    S::Encoded: Eq + Hash + std::fmt::Debug + Eq,
{
    type Plan = DStarPlan;
    fn plan(&mut self, env: &S, source: Coord, target: Coord) -> Option<Self::Plan> {
        println!("STARTED:: {source:?} -> {target:?}");
        if self.source.is_none() || self.target.is_none() || self.origin.is_none() {
            self.new_plan(env, source, target);
        } else {
            let s_encode = env.encode(source);
            self.k += env.distance(self.origin.unwrap(), s_encode);
            self.source = Some(s_encode);
            self.revise_plan(env);
            // self.new_plan(env, source, target);
        }
        match self.reconstruct_decode(env) {
            Some(plan) => Some(Self::Plan { plan }),
            None => None,
        }
    }
    fn update(&mut self, env: &S, position: Coord, obstacle: Coord) {
        if self.source.is_none() || self.target.is_none() {
            return;
        }
        let o_encode = env.encode(obstacle);
        let source = self.source.unwrap();
        let target = self.target.unwrap();
        let &(g_obs, rhs_obs) = self.star.get(&o_encode).unwrap_or(&(usize::MAX, usize::MAX));
        for neighbor in env.neighbors(o_encode) {
            if neighbor == target || neighbor == source || self.star.get(&neighbor).is_none() {
                continue;
            }
            let (g_n, rhs_n) = self.star[&neighbor];
            if rhs_n == env.distance(neighbor, o_encode).saturating_add(g_obs)
            {
                self.star.insert(neighbor, (usize::MAX, rhs_n));
                self.update_vertex(env, neighbor);
                self.propagate_g_invalidation(env, neighbor, g_n);
            }
        }
        self.update_vertex(env, o_encode);
        self.star.insert(o_encode, (usize::MAX, usize::MAX));
    }
}
