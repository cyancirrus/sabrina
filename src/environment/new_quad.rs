use crate::global::consts::LEVELS;
use crate::global::types::{ACoord, Belief, Bounds, HCoord};
use crate::hierarchy::new_encoding::{child_hier, encode, grid_hier};
use std::collections::HashMap;

type Information = HashMap<HCoord, QuadNode>;
// Sees in 4 principle components

#[derive(Eq, PartialEq, Copy, Clone, Debug)]
pub struct QuadNode {
    pub homogenous: bool,
    pub belief: Belief,
}
#[derive(Debug)]
pub struct QuadTree {
    pub information: Information,
    pub padding: Bounds,
    pub seen: Bounds,
    pub levels: usize,
}
impl QuadTree {
    pub fn new() -> Self {
        Self::initialize(LEVELS)
    }
    pub fn initialize(levels: usize) -> Self {
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
        let padding = Bounds {
            min_x: 0,
            min_y: 0,
            max_x: stride - 1,
            max_y: stride - 1,
        };
        let seen = Bounds {
            min_x: isize::MAX,
            min_y: isize::MAX,
            max_x: 0,
            max_y: 0,
        };
        Self {
            information,
            padding,
            seen,
            levels: levels,
        }
    }
    fn encode(&self, coord: ACoord) -> HCoord {
        HCoord {
            l: 0,
            x: coord.x,
            y: coord.y,
        }
    }
}

impl QuadTree {
    fn update_seen(&mut self, coord: ACoord) {
        if self.get_cell(coord).is_some() {
            return;
        }
        let span = 1 << (self.levels - 1);
        let node = encode(coord, self.levels - 1);
        self.padding.min_x = self.padding.min_x.min(node.x);
        self.padding.min_y = self.padding.min_y.min(node.y);
        self.padding.max_x = self.padding.max_x.max(node.x + span);
        self.padding.max_y = self.padding.max_y.max(node.y + span);
        self.seen.min_x = self.seen.min_x.min(node.x);
        self.seen.min_y = self.seen.min_y.min(node.y);
        self.seen.max_x = self.seen.max_x.max(node.x);
        self.seen.max_y = self.seen.max_y.max(node.y);
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
        self.update_seen(coord);
        self.set_cell(&coord, belief);
        self.bubble_belief(coord, belief);
        self.cleanse_repres(coord);
    }
    fn insert_known(&mut self, coord: ACoord, belief: Belief) -> bool {
        if let Some((_, current_belief)) = self.get_cell(coord) {
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
        if let Some((found_level, h_belief)) = self.get_cell(*coord) {
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
    pub fn get_cell(&self, coord: ACoord) -> Option<(usize, Belief)> {
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
    pub fn obstructed(&self, coord: ACoord) -> bool {
        match self.get_cell(coord) {
            Some((_, Belief::Free)) => true,
            Some((_, Belief::Unknown)) => true,
            Some((_, Belief::Occupied)) => false,
            None => false,
        }
    }
}
