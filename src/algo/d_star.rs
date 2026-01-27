use crate::environment::quad::QuadTree;
use crate::global::types::{Coord, DStarPlan, KeyHeap, KeyNode};
use crate::global::types::LazyPQueue;
use crate::hierarchy::encoding::centroid_estimate;
use crate::hierarchy::proximity::edge_neighbors;
use std::collections::{HashMap, HashSet};
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

pub type Star = HashMap<Coord, (G, Rhs)>;
/// Independent current estimate of cost to go
type G = usize;
// Estimate of cost given neighbors belief
type Rhs = usize;

// #[derive(Debug)]
// pub struct LazyPQueue {
//     heap: KeyHeap<Coord>,
//     lazy: HashSet<Coord>,
// }

// impl LazyPQueue {
//     pub fn new() -> Self {
//         Self {
//             heap: KeyHeap::new(),
//             lazy: HashSet::new(),
//         }
//     }
//     pub fn push(&mut self, node: KeyNode<Coord>) {
//         self.heap.push(node)
//     }
//     pub fn peek(&mut self) -> Option<&KeyNode<Coord>> {
//         loop {
//             let remove;
//             if let Some(node) = self.heap.peek() {
//                 if self.lazy.contains(&node.coord) {
//                     remove = true;
//                 } else {
//                     return self.heap.peek();
//                 }
//             } else {
//                 return None;
//             }
//             if remove {
//                 let node = self.heap.pop()?;
//                 self.lazy.remove(&node.coord);
//             }
//         }
//     }
//     pub fn remove(&mut self, coord: Coord) {
//         self.lazy.insert(coord);
//     }
//     pub fn pop(&mut self) -> Option<KeyNode<Coord>> {
//         while let Some(node) = self.heap.pop() {
//             if self.lazy.contains(&node.coord) {
//                 self.lazy.remove(&node.coord);
//             } else {
//                 return Some(node);
//             }
//         }
//         None
//     }
// }

fn update_vertex(
    quad: &QuadTree,
    star: &mut Star,
    update_queue: &mut LazyPQueue<Coord>,
    coord: Coord,
    target: Coord,
) {
    if coord == target {
        return;
    }
    let mut min_distance = usize::MAX;
    for neigh in edge_neighbors(&quad, coord) {
        let h = centroid_estimate(neigh, coord);
        if let Some(&(g, _rhs)) = star.get(&neigh) {
            min_distance = min_distance.min(g.saturating_add(h));
        }
    }
    match star.get_mut(&coord) {
        Some((g, rhs)) => {
            *rhs = min_distance;
            if g != rhs {
                let h = centroid_estimate(coord, target);
                update_queue.push(KeyNode::new(coord, *g, *rhs, h));
            } else {
                update_queue.remove(coord);
            }
        }
        None => {
            star.insert(coord, (usize::MAX, min_distance));
            update_queue.push(KeyNode {
                cost_astar: min_distance.saturating_add(centroid_estimate(coord, target)),
                cost_dijkstra: min_distance,
                coord,
            });
        }
    }
}

fn improve_and_invalidate(
    quad: &QuadTree,
    star: &mut Star,
    update_queue: &mut LazyPQueue<Coord>,
    target: Coord,
) {
    let u = update_queue.pop().unwrap();
    let (g_u, rhs_u) = star.get_mut(&u.coord).unwrap();
    if g_u > rhs_u {
        // improvement
        *g_u = *rhs_u;
    } else {
        // invalidation
        *g_u = usize::MAX;
        update_vertex(quad, star, update_queue, u.coord, target);
    }
    for p in edge_neighbors(quad, u.coord) {
        update_vertex(quad, star, update_queue, p, target);
    }
}

fn compute_shortest_path(
    quad: &QuadTree,
    star: &mut Star,
    update_queue: &mut LazyPQueue<Coord>,
    source: Coord,
    target: Coord,
) {
    star.insert(target, (usize::MAX, 0));
    star.insert(source, (centroid_estimate(source, target), usize::MAX));
    loop {
        match (star.get(&source), update_queue.peek()) {
            (Some(&(g, rhs)), Some(top_key)) => {
                let h = centroid_estimate(source, target);
                let start_key = KeyNode::new(source, g, rhs, h);
                // inverted compare logic because min-heap reverses compare
                if *top_key > start_key || g != rhs {
                    improve_and_invalidate(quad, star, update_queue, target);
                } else {
                    break;
                }
            }
            _ => break,
        }
    }
}

pub fn dstar_lite(
    quad: &QuadTree,
    star: &mut Star,
    update: &mut LazyPQueue<Coord>,
    source: Coord,
    target: Coord,
) -> DStarPlan {
    update.push(KeyNode::new(target, 0, 0, 0));
    compute_shortest_path(quad, star, update, source, target);
    let mut plan = Vec::new();
    let mut node_curr = Some(source);
    let mut node_next;
    while let Some(current) = node_curr {
        if current != source {
            plan.push(current);
        }
        if target == current {
            break;
        }
        node_next = None;
        let mut min_cost = usize::MAX;
        for neigh in edge_neighbors(quad, current) {
            if let Some(&(_g_n, rhs_n)) = star.get(&neigh) {
                let cost = rhs_n.saturating_add(centroid_estimate(current, neigh));
                if cost <= min_cost {
                    min_cost = cost;
                    node_next = Some(neigh);
                }
            }
        }
        mem::swap(&mut node_curr, &mut node_next);
    }
    DStarPlan { plan: plan }
}
