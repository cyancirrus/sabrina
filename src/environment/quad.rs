use crate::global::consts::{LEVELS, PARTITION};
use crate::global::types::{Belief, Bounds, Coord};
use crate::hierarchy::encoding::{child_hier, encode_hier, grid_hier};
use std::collections::HashMap;

type Information = HashMap<Coord, QuadNode>;
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
            encode_hier(&(0, 0), levels - 1),
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
            min_x: usize::MAX,
            min_y: usize::MAX,
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
    // pub fn initialize(information: Information, seen: Bounds, levels: usize) -> Self {
    //     Self {
    //         levels,
    //         seen,
    //         padding: Bounds {
    //             min_x: 0,
    //             min_y: 0,
    //             max_x: 0,
    //             max_y: 0,
    //         },
    //         information,
    //     }
    // }
}

impl QuadTree {
    pub fn update_seen(&mut self, coord: &Coord) {
        let m_coord = encode_hier(coord, self.levels - 1);
        let s_coord = (
            m_coord.0 & ((1 << PARTITION) - 1),
            m_coord.1 & ((1 << PARTITION) - 1),
        );
        self.padding.min_x = self.padding.min_x.min(s_coord.0);
        self.padding.min_y = self.padding.min_y.min(s_coord.1);
        self.padding.max_x = self.padding.max_x.max(s_coord.0 + (1 << (self.levels - 1)));
        self.padding.max_y = self.padding.max_y.max(s_coord.1 + (1 << (self.levels - 1)));
        self.seen.min_x = self.seen.min_x.min(coord.0);
        self.seen.min_y = self.seen.min_y.min(coord.1);
        self.seen.max_x = self.seen.max_x.max(coord.0);
        self.seen.max_y = self.seen.max_y.max(coord.1);
        if self.get_cell(coord).is_some() {
            return;
        }
        self.information.insert(
            m_coord,
            QuadNode {
                homogenous: true,
                belief: Belief::Unknown,
            },
        );
    }
    pub fn bubble_belief(&mut self, coord: &Coord, belief: Belief) {
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
            let m_coord = encode_hier(&coord, lvl + 1);
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
    pub fn cleanse_repres(&mut self, coord: &Coord) {
        let mut stack = Vec::new();
        for lvl in (1..self.levels).rev() {
            let m_coord = encode_hier(coord, lvl);
            if let Some(n) = self.information.get(&m_coord) {
                if n.homogenous {
                    for g in child_hier(&m_coord) {
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
    pub fn unknown(&mut self, coord: &Coord, belief: Belief) {
        self.update_seen(coord);
        self.set_cell(coord, belief);
        self.bubble_belief(coord, belief);
        self.cleanse_repres(coord);
    }
    pub fn known(&mut self, coord: &Coord, belief: Belief) -> bool {
        if let Some((_, current_belief)) = self.get_cell(coord) {
            // belief aligns
            return current_belief == belief;
        }
        if let Some(node) = self.information.get_mut(coord) {
            // align belief
            node.belief = belief;
            return true;
        }
        false
    }
    pub fn insert_cell(&mut self, coord: &Coord, belief: Belief) {
        if self.known(coord, belief) {
            return;
        }
        self.unknown(coord, belief);
    }
    pub fn split_cell(&mut self, coord: &Coord, level: usize) {
        // level > 0;
        let m_coord = encode_hier(coord, level);
        if let Some(ancestor) = self.information.remove(&m_coord) {
            for g in child_hier(&m_coord) {
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
    pub fn set_cell(&mut self, coord: &Coord, belief: Belief) {
        if let Some((level, h_belief)) = self.get_cell(coord) {
            if h_belief == belief {
                return;
            }
            for lvl in (1..=level).rev() {
                self.split_cell(coord, lvl);
            }
        }
        self.information.insert(
            *coord,
            QuadNode {
                belief,
                homogenous: true,
            },
        );
    }
    pub fn get_cell(&self, coord: &Coord) -> Option<(usize, Belief)> {
        for lvl in (0..self.levels).rev() {
            let m_coord = encode_hier(coord, lvl);
            if let Some(n) = self.information.get(&m_coord) {
                if n.homogenous {
                    return Some((lvl, n.belief));
                }
            }
        }
        None
    }
    pub fn path_clear(&self, coord: &Coord) -> bool {
        match self.get_cell(coord) {
            Some((_, Belief::Free)) => true,
            Some((_, Belief::Unknown)) => true,
            Some((_, Belief::Occupied)) => false,
            None => false,
        }
    }
}
