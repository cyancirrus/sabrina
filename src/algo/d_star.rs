#![allow(dead_code, unused_variables)]
use crate::global::types::LazyPQueue;
use crate::global::types::plan::Planner;
use crate::global::types::{Coord, DStarPlan, KeyNode, SpatialMap, StarKey};
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
    source: Option<S::Encoded>,
    target: Option<S::Encoded>,
}

impl<S: SpatialMap> DStarPlanner<S>
where
    S::Encoded: Eq + Hash + std::fmt::Debug + Eq,
{
    pub fn new() -> Self {
        Self {
            star: Star::new(),
            pqueue: LazyPQueue::new(),
            k: 0,
            source: None,
            target: None,
        }
    }
    fn initialize(&mut self, env: &S, source: S::Encoded, target: S::Encoded) {
        self.star.clear();
        self.pqueue.clear();
        self.k = 0;
        self.source = Some(source);
        self.target = Some(target);
        let h = env.distance(source, target);
        self.star.insert(target, (h, 0));
        self.star.insert(source, (usize::MAX, usize::MAX));
        self.pqueue.push(KeyNode {
            star_key: StarKey {
                cost_astar: h,
                cost_dijkstra: 0,
            },
            coord: target,
        });
    }
    fn update_vertex(&mut self, env: &S, u: S::Encoded) {
        let source =self.source.unwrap();
        let target =self.target.unwrap();
        if u == target { return; }
        if let Some((g, rhs)) = self.star.get(&u) {
            if g != rhs {
                let key = StarKey::new(*g, *rhs, env.distance(u, source), self.k);
                self.pqueue.push(KeyNode {
                    star_key: key,
                    coord: u,
                });
            } else {
                self.pqueue.remove(u);
            }
        }
    }
    fn propogate_cost_rhs(&mut self, env: &S, u: S::Encoded) {
        let target = self.target.unwrap();
        let (g_u, _) = self.star[&u];
        for s in env.neighbors(u) {
            if s == target {
                continue;
            }
            let rhs_new = env.distance(s, u).saturating_add(g_u);
            if let Some((g, rhs)) = self.star.get_mut(&s) {
                *rhs = (*rhs).min(rhs_new);
            } else {
                self.star.insert(s, (usize::MAX, rhs_new));
            }
            self.update_vertex(env, s);
        }
    }
    fn find_min_neighbor_g(&self, env: &S, s: S::Encoded) -> usize {
        let target = self.target.unwrap();
        let mut min_cost = usize::MAX;
        for n in env.neighbors(s) {
            if n == target { continue; }
            if let Some(&(g, _)) = self.star.get(&n) {
                min_cost = min_cost.min(env.distance(s, n).saturating_add(g));
            }
        }
        min_cost
    }
    fn propogate_cost_g(&mut self, env: &S,  u:S::Encoded, g_old: usize) {
        let target = self.target.unwrap();
        for n in env.neighbors(u) {
            if n == target {
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
            self.star.insert(n, (g, rhs_new));
            self.update_vertex(env, n);
        }
    }
    fn update_costs(&mut self, env: &S, u: S::Encoded) {
        if u == self.target.unwrap() || self.star.get(&u).is_none() {
            return;
        }
        let (g_old, r_old) = self.star[&u];
        self.propogate_cost_g(env, u, g_old);
        self.propogate_cost_rhs(env, u);
        self.update_vertex(env, u);
        
    }
    fn compute_shortest_path(&mut self, env: &S) {
        let source = self.source.unwrap();
        let target = self.target.unwrap();
        loop {
            let (g, rhs) = self.star[&source];
            if let Some(top_key_node) = self.pqueue.peek() {
                let top_key= top_key_node.star_key;
                let start_key = StarKey::new(g, rhs, env.distance(source, target), self.k);
                if g == rhs && top_key < start_key {
                    break;
                }
            }
            let u = self.pqueue.pop().unwrap();
            let &(g_u, rhs_u) = match self.star.get(&u.coord) {

                Some(entry) => entry,
                None => continue,
            };
            let k_new = StarKey::new(g_u, rhs_u, env.distance(source, u.coord), self.k);
            // rversed due to min heap
            if u.star_key > k_new  {
                self.pqueue.push(KeyNode {
                    star_key: k_new,
                    coord: u.coord,
                });
            } else if g_u > rhs_u {
                self.star.insert(u.coord, (rhs_u, rhs_u));
                self.pqueue.remove(u.coord);
                self.propogate_cost_rhs(env, u.coord);
            } else if g_u < rhs_u {
                let g_old = g_u;
                self.star.insert(u.coord, (usize::MAX, rhs_u));
                self.propogate_cost_g(env, u.coord, g_old);
            }
        }
    }
    fn reconstruct_decode(
        &mut self,
        env: &S,
    ) -> Option<Vec<Coord>> {
        // assert!(false);
        let source = self.source.unwrap();
        let target = self.target.unwrap();
        let mut plan = Vec::new();
        let mut node_curr = Some(source);
        let mut node_next;
        let mut best_cost;
        while let Some(current) = node_curr {
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
                    let cost = g_n;
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

    fn new_plan(&mut self, env: &S, source: Coord, target: Coord){
        if env.obstructed(target) {
            return
        };
        let s_encode = env.encode(source);
        let t_encode = env.encode(target);
        // encodes distance matrix
        self.initialize(env, s_encode, t_encode);
        self.compute_shortest_path(env);
    }
    fn revise_plan(&mut self, env:&S) {
        let source =self.source.unwrap();
        let target =self.target.unwrap();
        let h = env.distance(self.source.unwrap(), self.target.unwrap());
        // self.star.insert(target, (h, 0));
        self.star.insert(source, (usize::MAX, usize::MAX));
        self.compute_shortest_path(env);

    }
}

impl<S: SpatialMap> Planner<S> for DStarPlanner<S>
where
    S::Encoded: Eq + Hash + std::fmt::Debug + Eq,
{
    type Plan = DStarPlan;
    fn plan(&mut self, env: &S, source:Coord, target: Coord) -> Option<Self::Plan> {
        println!("STARTED:: {source:?} -> {target:?}");
        if self.source.is_none() || self.target.is_none() {
            self.new_plan(env, source, target);
        } else {
            // let s_encode = env.encode(source);
            // self.k += env.distance(s_encode, self.source.unwrap());
            // self.source = Some(s_encode);
            // self.revise_plan(env);
            self.new_plan(env, source, target);
        }
        match self.reconstruct_decode(env) {
            Some(plan) => { Some(Self::Plan { plan }) },
            None => None
        }
    }
    fn update(&mut self, env: &S, obstacle: Coord) {
        if self.source.is_none() || self.target.is_none() {
            return;
        }
        let o_encode = env.encode(obstacle);
        self.star.insert(o_encode, (usize::MAX, usize::MAX));
        
        self.update_costs(env, o_encode);
        for neighbor in env.neighbors(o_encode) {
            self.update_costs(env, neighbor);
        }
    }
}
