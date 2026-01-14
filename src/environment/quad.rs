use crate::environment::morton::{child_morton, encode_morton, grid_morton};
use crate::global::consts::{GRID, LEVELS};
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
    bounds: Bounds,
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
                max_x: 0,
                min_y: 0,
                max_y: 0,
            },
            information: HashMap::new(),
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
            self.information.insert(
                m_coord,
                QuadNode {
                    homogenous: false,
                    belief: Belief::Unknown,
                },
            );
        }
    }
    pub fn get_grid(&self, coord: &Coord, level: isize) -> Option<[QuadNode; GRID]> {
        let grid = grid_morton(coord, level);
        for d in grid {
            if !self.information.contains_key(&d) {
                return None;
            };
        }
        Some([
            self.information[&grid[0]],
            self.information[&grid[1]],
            self.information[&grid[2]],
            self.information[&grid[3]],
        ])
    }

    pub fn bubble_belief(&mut self, coord: &Coord, mut belief: Belief) {
        for lvl in 0..self.levels {
            let m_coord = encode_morton(coord, lvl);
            if let Some(n) = self.information.get_mut(&m_coord) {
                // homogenous as previous level for all grid memberew homogenous
                n.homogenous = true;
                n.belief = belief;
            }
            if let Some(grid) = self.get_grid(&coord, lvl) {
                belief = grid[0].belief;
                for i in 1..GRID {
                    if grid[i].belief != belief {
                        return;
                    }
                }
            } else {
                return;
            }
        }
    }
    pub fn cleanse_repres(&mut self, coord: &Coord) {
        let mut stack = Vec::new();
        for lvl in (1..LEVELS).rev() {
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
            if lvl == 0 {
                continue;
            }
            for g in child_morton(&m) {
                stack.push((lvl - 1, g));
            }
        }
    }
    pub fn insert_cell(&mut self, coord: &Coord, belief: Belief) {
        self.update_bounds(coord);
        self.insert_levels(coord);
        self.bubble_belief(coord, belief);
        self.cleanse_repres(coord);
    }
    fn get_cell(&self, coord: &Coord) -> Option<Belief> {
        for lvl in (0..self.levels).rev() {
            let m_coord = encode_morton(coord, lvl);
            if let Some(n) = self.information.get(&m_coord) {
                if n.homogenous {
                    return Some(n.belief);
                }
            }
        }
        None
    }
}

impl QuadTree {
    fn get_cell_with_level(&self, coord: &Coord) -> Option<(isize, Belief)> {
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
    pub fn display_with_levels(&self) {
        for i in (self.bounds.min_y..=self.bounds.max_y).rev() {
            let mut line = String::new();
            for j in self.bounds.min_x..=self.bounds.max_x {
                match self.get_cell_with_level(&(j, i)) {
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
                    Some(Belief::Occupied) => '#',
                    Some(Belief::Unknown) => '?',
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
