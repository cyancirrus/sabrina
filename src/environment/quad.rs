use crate::environment::morton::{child_morton, encode_morton, grid_morton};
use crate::global::consts::LEVELS;
use crate::global::types::{Bounds, Coord};
use std::collections::HashMap;
use std::fmt;

type Information = HashMap<Coord, QuadNode>;
// Sees in 4 principle components

#[derive(Eq, PartialEq, Copy, Clone, Debug)]
pub struct QuadNode {
    homogenous: bool,
    belief: Belief,
}

#[derive(Debug)]
pub struct QuadTree {
    levels: isize,
    pub bounds: Bounds,
    information: Information,
}
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
pub enum Belief {
    Free,
    Occupied,
    Unknown,
}

impl QuadTree {
    pub fn new() -> Self {
        Self {
            levels: LEVELS,
            bounds: Bounds {
                min_x: 0,
                min_y: 0,
                max_x: (1 << LEVELS-1) - 1,
                max_y: (1 << LEVELS-1) - 1,
            },
            information: HashMap::from([(
                encode_morton(&(0, 0), LEVELS - 1),
                QuadNode {
                    belief: Belief::Unknown,
                    homogenous: true,
                },
            )]),
        }
    }
    pub fn initialize(information: Information, bounds: Bounds, levels: isize) -> Self {
        Self {
            levels,
            bounds,
            information,
        }
    }
}

impl QuadTree {
    pub fn update_bounds(&mut self, coord: &Coord) {
        self.bounds.min_x = self.bounds.min_x.min(coord.0);
        self.bounds.min_y = self.bounds.min_y.min(coord.1);
        self.bounds.max_x = self.bounds.max_x.max(coord.0);
        self.bounds.max_y = self.bounds.max_y.max(coord.1);
    }
    pub fn insert_levels(&mut self, coord: &Coord) {
        self.information.insert(
            *coord,
            QuadNode {
                homogenous: true,
                belief: Belief::Occupied,
            },
        );
        for lvl in 1..self.levels {
            let m_coord = encode_morton(coord, lvl);
            if self.information.contains_key(&m_coord) {
                return;
            }
            self.information.insert(
                m_coord,
                QuadNode {
                    homogenous: false,
                    belief: Belief::Unknown,
                },
            );
        }
    }
    pub fn bubble_belief(&mut self, coord: &Coord, belief: Belief) {
        for lvl in 0..self.levels - 1 {
            for g in grid_morton(&coord, lvl) {
                if let Some(qnode) = self.information.get(&g) {
                    if qnode.belief != belief {
                        return;
                    }
                } else {
                    return;
                }
            }
            let m_coord = encode_morton(&coord, lvl + 1);
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
            let m_coord = encode_morton(coord, lvl);
            if let Some(n) = self.information.get(&m_coord) {
                if n.homogenous {
                    for g in child_morton(&m_coord) {
                        stack.push((lvl - 1, g));
                    }
                    break;
                }
            } else {
                return;
            }
        }
        while let Some((lvl, m)) = stack.pop() {
            self.information.remove(&m);
            if lvl > 0 {
                for g in child_morton(&m) {
                    stack.push((lvl - 1, g));
                }
            }
        }
    }
    pub fn insert_cell(&mut self, coord: &Coord, belief: Belief) {
        self.update_bounds(coord);
        self.insert_levels(coord);
        self.bubble_belief(coord, belief);
        self.cleanse_repres(coord);
    }
    pub fn split_cell(&mut self, coord: &Coord, belief: Belief, level: isize) {
        if level == 0 {
            return;
        }
        let m_coord = encode_morton(coord, level);
        let n_coord = encode_morton(coord, level - 1);
        let Some(parent_node) = self.information.remove(&m_coord) else {
            return;
        };
        for g in child_morton(coord) {
            let belief = if g == n_coord {
                belief
            } else {
                parent_node.belief
            };
            self.information.insert(
                g,
                QuadNode {
                    belief,
                    homogenous: parent_node.homogenous,
                },
            );
        }
    }
    pub fn set_cell(&mut self, coord: &Coord, belief: Belief) {
        if let Some((level, h_belief)) = self.get_cell(coord) {
            if h_belief == belief {
                return;
            }
            for lvl in (1..level).rev() {
                self.split_cell(coord, belief, lvl);
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
    fn get_cell(&self, coord: &Coord) -> Option<(isize, Belief)> {
        for lvl in (0..self.levels).rev() {
            let m_coord = encode_morton(coord, lvl);
            if let Some(n) = self.information.get(&m_coord) {
                if n.homogenous {
                    return Some((lvl, n.belief));
                }
            }
        }
        None
    }
}

impl QuadTree {
    pub fn display_with_levels(&self) {
        for i in (self.bounds.min_y..=self.bounds.max_y).rev() {
            let mut line = String::new();
            for j in self.bounds.min_x..=self.bounds.max_x {
                match self.get_cell(&(j, i)) {
                    None => line.push_str("[ ]"),
                    Some((lvl, Belief::Occupied)) => line.push_str(&format!("[{lvl:}]")),
                    Some((lvl, Belief::Unknown)) => line.push_str(&format!("[{lvl:}]")),
                    Some((lvl, Belief::Free)) => line.push_str(&format!("[{lvl:}]")),
                };
            }
            println!("{}", line);
        }
    }
}

impl fmt::Display for QuadTree {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for i in (self.bounds.min_y..=self.bounds.max_y).rev() {
            let mut line = String::new();
            for j in self.bounds.min_x..=self.bounds.max_x {
                let symbol = match self.get_cell(&(j, i)) {
                    None => ' ',
                    Some((_, Belief::Occupied)) => '#',
                    Some((_, Belief::Unknown)) => '?',
                    Some(_) => '?',
                };
                line.push('[');
                line.push(symbol);
                line.push(']');
            }
            let _ = writeln!(f, "{}", line);
        }
        Ok(())
    }
}
