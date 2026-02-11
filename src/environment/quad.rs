#![allow(unused_imports)]
use crate::global::consts::LEVELS;
use crate::global::types::{ACoord, Belief, Bounds, HCoord, SpatialMap};
use crate::hierarchy::encoding::{child_hier, encode, grid_hier, point, transform};
use crate::hierarchy::proximity::{edge_neighbors, grid_children, grid_leaf, grid_siblings};
use std::collections::HashMap;

type Information = HashMap<HCoord, QuadNode>;
// Sees in 4 principle components

#[derive(Eq, PartialEq, Copy, Clone, Debug)]
pub struct QuadNode {
    pub homogenous: bool,
    pub belief: Belief,
}
#[derive(Clone)]
pub struct QuadTree {
    pub information: Information,
    pub bounds: Bounds,
    pub levels: usize,
}

impl SpatialMap for QuadTree {
    type Encoded = HCoord;
    fn encode(&self, coord: ACoord) -> Self::Encoded {
        for lvl in 0..self.levels {
            let node = encode(coord, lvl);
            if self.information.contains_key(&node) {
                return node;
            }
        }
        let l = self.levels - 1;
        let m = !((1 << l) - 1);
        HCoord {
            l: l,
            x: coord.x & m,
            y: coord.y & m,
        }
    }
    fn decode(&self, node: Self::Encoded) -> ACoord {
        ACoord {
            x: node.x,
            y: node.y,
        }
    }
    fn leaf(&self, coord: ACoord) -> Self::Encoded {
        HCoord {
            l: 0,
            x: coord.x,
            y: coord.y,
        }
    }
    fn initialize(&mut self, source: ACoord, target: ACoord) {
        // TODO: Needs some sort of intelligent resizing
        // ensure the the grid has been initialized
        // let span = 1 << (self.levels - 1);
        let span = 2;
        self.bounds.min_x = target.x.min(source.x).min(self.bounds.min_x);
        self.bounds.min_y = target.y.min(source.y).min(self.bounds.min_y);
        self.bounds.max_x = target.x.max(source.x).max(self.bounds.max_x);
        self.bounds.max_y = target.y.max(source.y).max(self.bounds.max_y);
        for x in (self.bounds.min_x..=self.bounds.max_x).step_by(span as usize) {
            for y in (self.bounds.min_y..=self.bounds.max_y).step_by(span as usize) {
                let node = self.encode(ACoord { x, y });
                self.populate_edge(node)
            }
        }
    }
    fn obstructed(&self, coord: ACoord) -> bool {
        match self.get_coord(coord) {
            Some((_, Belief::Free)) => false,
            Some((_, Belief::Unknown)) => false,
            Some((_, Belief::Occupied)) => true,
            None => false,
        }
    }
    fn distance(&self, a: Self::Encoded, b: Self::Encoded) -> usize {
        // distance between source-centroid and target-centroid
        // ----------------
        // let p = (1 << a.l) + (1<< b.l);
        // let p = a.l.max(b.l);
        // let (a, b) = (point(a), point(b));
        // a.x.abs_diff(b.x) + a.y.abs_diff(b.y) + (1 << p)
        // ----------------
        // let (a, b) = (point(a), point(b));
        // ((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)) as usize
        let (a, b) = (point(a), point(b));
        a.x.abs_diff(b.x) + a.y.abs_diff(b.y)
    }
    fn neighbors(&self, a: Self::Encoded) -> Vec<Self::Encoded> {
        edge_neighbors(self, a)
    }
    fn retrieve(&self, mut node: Self::Encoded) -> Self::Encoded {
        for lvl in 0..self.levels {
            node = transform(&node, lvl);
            if self.information.contains_key(&node) {
                return node;
            }
        }
        node
    }
    fn components(&self, node: &Self::Encoded) -> Vec<Self::Encoded> {
        match node.l {
            0 => vec![],
            _ => grid_children(node).to_vec(),
        }
    }
    fn belief(&self, node: Self::Encoded) -> Belief {
        match self.get_node(node) {
            Some((_, belief)) => belief,
            None => Belief::Unknown,
        }
    }
    fn insert_ray(&mut self, mut pos: ACoord, hit: ACoord) {
        // beliefs not recorded are assumed unknown
        // handles simulation compass rose signals
        let (dy, dx) = (hit.y - pos.y, hit.x - pos.x);
        let (del_y, del_x) = (dy.signum(), dx.signum());
        while pos != hit {
            // NOTE: Cant do this until i have a way to track splits
            // self.update_belief(&pos, Belief::Free);
            pos.x += del_x;
            pos.y += del_y;
        }
        self.update_belief(&hit, Belief::Occupied);
    }
}

impl QuadTree {
    pub fn new() -> Self {
        Self::init(LEVELS)
    }
    pub fn init(levels: usize) -> Self {
        let mut information = HashMap::new();
        // level 0 contains no shift and level is inclusive
        let stride = 1 << (levels - 1);
        information.insert(
            encode(ACoord { x: 0, y: 0 }, levels - 1),
            QuadNode {
                belief: Belief::Unknown,
                homogenous: true,
            },
        );
        let bounds = Bounds {
            min_x: 0,
            min_y: 0,
            max_x: stride - 1,
            max_y: stride - 1,
        };
        Self {
            information,
            bounds,
            levels: levels,
        }
    }
}

impl QuadTree {
    pub fn populate_edge(&mut self, node: HCoord) {
        // if self.get_node(node).is_some() {
        //     return;
        // }
        let span = 1 << (self.levels - 1);
        let node = transform(&node, self.levels - 1);
        self.bounds.min_x = self.bounds.min_x.min(node.x);
        self.bounds.min_y = self.bounds.min_y.min(node.y);
        self.bounds.max_x = self.bounds.max_x.max(node.x + span - 1);
        self.bounds.max_y = self.bounds.max_y.max(node.y + span - 1);
        self.information.insert(
            node,
            QuadNode {
                homogenous: true,
                belief: Belief::Unknown,
            },
        );
    }
    fn update_bounds(&mut self, coord: ACoord) {
        if self.get_coord(coord).is_some() {
            return;
        }
        let span = 1 << (self.levels - 1);
        let node = encode(coord, self.levels - 1);
        self.bounds.min_x = self.bounds.min_x.min(node.x);
        self.bounds.min_y = self.bounds.min_y.min(node.y);
        self.bounds.max_x = self.bounds.max_x.max(node.x + span - 1);
        self.bounds.max_y = self.bounds.max_y.max(node.y + span - 1);
        self.information.insert(
            node,
            QuadNode {
                homogenous: true,
                belief: Belief::Unknown,
            },
        );
    }
    fn bubble_belief(&mut self, coord: ACoord, belief: Belief) {
        for lvl in 0..self.levels - 1 {
            for g in grid_hier(&coord, lvl) {
                if let Some(qnode) = self.information.get(&g) {
                    if qnode.belief != belief {
                        return;
                    }
                } else {
                    return;
                }
            }
            let m_coord = encode(coord, lvl + 1);
            if let Some(ancestor) = self.information.get_mut(&m_coord) {
                ancestor.homogenous = true;
                ancestor.belief = belief;
            } else {
                self.information.insert(
                    m_coord,
                    QuadNode {
                        homogenous: true,
                        belief: belief,
                    },
                );
            }
        }
    }
    fn cleanse_repres(&mut self, node: ACoord) {
        let mut stack = Vec::new();
        for lvl in (1..self.levels).rev() {
            let node = encode(node, lvl);
            if let Some(n) = self.information.get(&node) {
                if n.homogenous {
                    for g in child_hier(&node) {
                        stack.push((lvl - 1, g));
                    }
                    break;
                }
            }
        }
        while let Some((lvl, m)) = stack.pop() {
            self.information.remove(&m);
            if lvl > 0 {
                for g in child_hier(&m) {
                    stack.push((lvl - 1, g));
                }
            }
        }
    }
    fn insert_unknown(&mut self, coord: ACoord, belief: Belief) {
        self.update_bounds(coord);
        self.set_cell(&coord, belief);
        self.bubble_belief(coord, belief);
        self.cleanse_repres(coord);
    }
    fn insert_known(&mut self, coord: ACoord, belief: Belief) -> bool {
        if let Some((_, current_belief)) = self.get_coord(coord) {
            // belief aligns
            return current_belief == belief;
        }
        let node = HCoord {
            l: 0,
            x: coord.x,
            y: coord.y,
        };
        if let Some(node) = self.information.get_mut(&node) {
            // align belief
            node.belief = belief;
            return true;
        }
        false
    }
    pub fn update_belief(&mut self, coord: &ACoord, belief: Belief) {
        self.update_bounds(*coord);
        if self.insert_known(*coord, belief) {
            return;
        }
        self.insert_unknown(*coord, belief);
    }
    fn split_cell(&mut self, node: HCoord) {
        if let Some(ancestor) = self.information.remove(&node) {
            for g in child_hier(&node) {
                self.information.insert(
                    g,
                    QuadNode {
                        belief: ancestor.belief,
                        homogenous: ancestor.homogenous,
                    },
                );
            }
        };
    }
    fn set_cell(&mut self, coord: &ACoord, belief: Belief) {
        if let Some((found_level, h_belief)) = self.get_coord(*coord) {
            if h_belief == belief {
                return;
            }
            for lvl in (1..=found_level).rev() {
                self.split_cell(encode(*coord, lvl));
            }
        }
        self.information.insert(
            HCoord {
                l: 0,
                x: coord.x,
                y: coord.y,
            },
            QuadNode {
                belief,
                homogenous: true,
            },
        );
    }
    pub fn retrieve_node(&self, mut node: HCoord) -> Option<(usize, Belief)> {
        for lvl in 0..self.levels {
            node = transform(&node, lvl);
            if let Some(n) = self.information.get(&node) {
                if n.homogenous {
                    return Some((lvl, n.belief));
                }
            }
        }
        None
    }
    pub fn get_node(&self, mut node: HCoord) -> Option<(usize, Belief)> {
        for lvl in 0..self.levels {
            node = transform(&node, lvl);
            if let Some(n) = self.information.get(&node) {
                if n.homogenous {
                    return Some((lvl, n.belief));
                }
            }
        }
        None
    }
    pub fn get_coord(&self, coord: ACoord) -> Option<(usize, Belief)> {
        for lvl in 0..self.levels {
            let node = encode(coord, lvl);
            if let Some(n) = self.information.get(&node) {
                if n.homogenous {
                    return Some((lvl, n.belief));
                }
            }
        }
        None
    }
}

#[cfg(test)]
pub mod test_distance {
    use super::*;
    #[test]
    fn test_distance_metric() {
        let env = QuadTree::new();
        let a = HCoord{l:0, x: 0, y: 0};
        let b = HCoord{l:0, x: 1, y: 1};
        assert_eq!(4, env.distance(a, b));
        
        let a = HCoord{l:0, x: 0, y: 0};
        let b = HCoord{l:0, x: 2, y: 0};
        assert_eq!(4, env.distance(a, b));
        
        let a = HCoord{l:1, x: 0, y: 0};
        let b = HCoord{l:0, x: 0, y: 0};
        assert_eq!(2, env.distance(a, b));
        
        let a = HCoord{l:0, x: 0, y: 0};
        let b = HCoord{l:0, x: 8, y: 0};
        assert_eq!(16, env.distance(a, b));
        
        let a = HCoord{l:0, x: 0, y: 0};
        let b = HCoord{l:0, x: 8, y: 8};
        assert_eq!(32, env.distance(a, b));
    }
}
