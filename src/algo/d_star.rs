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
        self.origin = Some(source);
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
        let origin = self.origin.unwrap();
        let source = self.source.unwrap();
        let target = self.target.unwrap();
        let h = env.distance(u, source);
        let &(g, rhs) = self.star.get(&u).unwrap_or(&(usize::MAX, usize::MAX));
        StarKey::new(g, rhs, h, self.k)
    }
    fn update_vertex(&mut self, env: &S, u: S::Encoded) {
        let source = self.source.unwrap();
        let (g, rhs) = self.star.get(&u).unwrap_or(&(usize::MAX, 0));
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
        let target = self.target.unwrap();
        let mut min_cost = usize::MAX;
        for s_p in env.neighbors(s) {
            if let Some(&(g_sp, _)) = self.star.get(&s_p) {
                min_cost = min_cost.min(env.distance(s, s_p).saturating_add(g_sp));
            }
        }
        min_cost
    }
    // fn propagate_cost_g(&mut self, env: &S, u: S::Encoded, g_old: usize) {
    //     let target = self.target.unwrap();
    //     if u == target {
    //         return;
    //     }
    //     for s in env.neighbors(u) {
    //         // only update if not equal
            
    //         let &(g_s, rhs) = self.star.get(&s).unwrap_or(&(usize::MAX, 0));
    //         if rhs != g_s  { continue; } 
    //         let rhs_new = self.find_min_neighbor_g(env, s);
    //         self.star.insert(s, (g_s, rhs_new));
    //         self.update_vertex(env, s);
    //     }
    // }
    fn propagate_cost_g(&mut self, env: &S, u: S::Encoded, g_old: usize) {
        let target = self.target.unwrap();
        if u == target {
            return;
        }
        for s in env.neighbors(u) {
            // only update if not equal
            let g = if let Some(&(g_s, rhs)) = self.star.get(&s) {
                if rhs != env.distance(u, s).saturating_add(g_old) {
                    continue;
                }
                g_s
            } else {
                continue;
            };
            let rhs_new = self.find_min_neighbor_g(env, s);
            self.star.insert(s, (g, rhs_new));
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
                    println!("START KEY WAS {start_key:?}");
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
        println!("DECODING");
        println!("{:?}", self.pqueue);
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
        self.origin = Some(s_encode);
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
        // let (g, rhs) = self.star[&source];
        let (g, rhs) = self.star[&source];
        self.update_vertex(env, source);
        self.compute_shortest_path(env);
    }
}

impl<S: SpatialMap> Planner<S> for DStarPlanner<S>
where
    S::Encoded: Eq + Hash + std::fmt::Debug + Eq,
{
    type Plan = DStarPlan;
    fn plan(&mut self, env: &S, source: ACoord, target: ACoord) -> Option<Self::Plan> {
        println!("ROUTING (source:{source:?}) -> (target: {target:?})");
        println!("-----------------------");
        println!("{:?}", self.star);
        println!("-----------------------");
        if self.source.is_none() || self.target.is_none() || self.origin.is_none() {
            self.new_plan(env, source, target);
        } else {
            let s_encode = env.encode(source);
            let t_encode = env.encode(target);
            println!("target {:?}", self.target.unwrap());

            self.k += env.distance(self.origin.unwrap(), s_encode);
            self.source = Some(s_encode);
            self.revise_plan(env);
        }
        match self.reconstruct_decode(env) {
            Some(plan) => Some(Self::Plan { plan }),
            None => None,
        }
    }
    fn update(&mut self, env: &S, position: ACoord, obstacle: ACoord) {
        // TODO: Need to integrate rhs for when we find better paths
        if self.source.is_none() || self.target.is_none() {
            return;
        }
        let node = env.encode(obstacle);
        let leaf = env.leaf(obstacle);
        let source = self.source.unwrap();
        let target = self.target.unwrap();
        // if worse
        let &(g_obs, rhs_obs) = self
            .star
            .get(&node)
            .unwrap_or(&(usize::MAX, usize::MAX));
        self.star.insert(node, (usize::MAX, usize::MAX));
        self.star.insert(leaf, (usize::MAX, usize::MAX));
        // self.propagate_cost_g(env, node, g_obs);
        // self.update_vertex(env, node);
        self.propagate_cost_g(env, leaf, g_obs);
        self.update_vertex(env, leaf);
        // // if better
        // self.star.insert(u_coord, (rhs_u, rhs_u));
        // self.pqueue.remove(&u_coord);
        // self.propagate_cost_rhs(env, u_coord);
    }
}

// fn main() {
//     use sabrina::environment::grid::Grid;
//     // use sabrina::algo::a_star::AStarPlanner;
//     // use sabrina::algo::best_first::BestFirstPlanner;
//     use sabrina::algo::d_star::DStarPlanner;
//     use sabrina::intelligence::sabrina::Sabrina;
//     use sabrina::parser::grid::read_grid;
//     use sabrina::sensor::lidar::Lidar;
//     println!("------------------------------------");
//     println!("      Example navigation            ");
//     println!("------------------------------------");
//     let path = "./data/sample/test_nav0.map";
//     // let path = "./data/sample/test_nav1.map";
//     // let path = "./data/sample/test_imposs.map";
//     match read_grid(path) {
//         Ok(oracle) => {
//             // let position = (1, 1);
//             // let target = (1, 3);

//             // let position = (1, 1);
//             // let target = (9, 3);

//             // let position = (4, 1);
//             // let target = (4, 3);

//             let position = (1, 1);
//             let target = (18, 3);

//             let environment = Grid::new();
//             let lidar = Lidar::new(6, oracle.clone());
//             // let mut sabby = Sabrina::new(position, environment, lidar, BestFirstPlanner);
//             // let mut sabby = Sabrina::new(position, environment, lidar, AStarPlanner);
//             // let mut sabby = Sabrina::new(position, oracle.clone(), lidar, DStarPlanner::new());
//             let mut sabby = Sabrina::new(position, environment, lidar, DStarPlanner::new());
//             println!("absolute_environment\n{oracle}");
//             println!("-------------------------------");
//             println!("    Starting Navigation        ");
//             println!("-------------------------------");
//             println!("Final Status {:?}", sabby.navigate(target));
//             println!("Final map\n{}", sabby.environment);
//         }
//         Err(e) => {
//             println!("Err\n{e:?}");
//         }
//     }
// }
