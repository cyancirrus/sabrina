#![allow(unused)]
use sabrina::environment::representation::{Bounds, Environment};
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::map::readmap;
use sabrina::sensor::lidar::Lidar;
use std::collections::HashMap;
use std::error::Error;
use std::fmt;
use std::fs;

const LEVELS: isize = 3;
const GRID: usize = 4;
const PARTITION: usize = 32;
type Coord = (isize, isize);
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

pub fn readquad(path: &str, levels: isize) -> Result<QuadTree, Box<dyn Error>> {
    let content = match fs::read_to_string(path) {
        Ok(c) => c,
        Err(e) => return Err(format!("Unable to read path {path:?}\n{e:?}").into()),
    };
    let (mut max_x, mut max_y) = (0, 0);
    let mut mirrored_objects = Vec::new();
    for (idx_y, line) in content.lines().enumerate() {
        for (idx_x, cell) in line.as_bytes().chunks_exact(3).enumerate() {
            let obj = match cell[1] {
                b' ' => continue,
                b'+' => Belief::Occupied,
                b'*' => Belief::Occupied,
                b'#' => Belief::Occupied,
                b'x' => Belief::Occupied,
                _ => {
                    return Err(
                        format!("Unexpected symbol found in map with source {path:?}").into(),
                    );
                }
            };
            mirrored_objects.push(((idx_x as isize, idx_y as isize), obj));
            max_x = max_x.max(idx_x as isize);
        }
        max_y = max_y.max(idx_y as isize);
    }
    let mut information = HashMap::new();
    let bounds = Bounds::new(0, 0, max_x, max_y);
    let mut quadtree = QuadTree::initialize(information, bounds, levels);
    // Mapping is easiest to think of as direct representation ie mirrored b/c of parsing
    for ((idx_x, mir_idx_y), obj) in mirrored_objects {
        quadtree.insert_cell(&(idx_x, max_y - mir_idx_y), obj);
    }

    Ok(quadtree)
}

fn encode_morton(coord: &Coord, level: isize) -> (Coord) {
    debug_assert!(
        coord.0 < (1 << PARTITION)
            && coord.1 < (1 << PARTITION)
            && level < (1 << isize::BITS as usize - PARTITION)
    );
    let mask = !((1 << level) - 1);
    (
        (coord.0 & mask) | (level << PARTITION),
        (coord.1 & mask) | (level << PARTITION),
    )
}

fn child_morton(morton: &Coord) -> [Coord;4] {
    let level = (morton.0 >> PARTITION) - 1;
    // something off by one here
    // let mask = ((1<<PARTITION+1) -1);
    // println!("mask {:b}", mask);
    // println!("before shifting {:}", level + 1);
    // println!("the mask should {:b}", mask & morton.0);
    [
        (
            (morton.0 - (1<< PARTITION)),
            (morton.1 - (1 << PARTITION)),
        ),
        (
            (morton.0 - (1 << PARTITION))  | 1 << level,
            (morton.1 - (1 << PARTITION)) ,
        ),
        (
            (morton.0 - (1 << PARTITION)) ,
            (morton.1 - (1 << PARTITION))  | 1 << level,
        ),
        (
            (morton.0 - (1 << PARTITION))  | 1 << level,
            (morton.1 - (1 << PARTITION))  | 1 << level,
        ),
    ]

}

fn grid_morton(coord: &Coord, level: isize) -> [Coord; 4] {
    let mask = !((1 << (level + 1)) - 1);
    // clockwise navigation through grid
    [
        (
            (coord.0 & mask) | (level << PARTITION),
            (coord.1 & mask) | (level << PARTITION),
        ),
        (
            (coord.0 & mask) | (level << PARTITION) | 1 << level,
            (coord.1 & mask) | (level << PARTITION),
        ),
        (
            (coord.0 & mask) | (level << PARTITION),
            (coord.1 & mask) | (level << PARTITION) | 1 << level,
        ),
        (
            (coord.0 & mask) | (level << PARTITION) | 1 << level,
            (coord.1 & mask) | (level << PARTITION) | 1 << level,
        ),
    ]
}

fn print_morton(morton: &Coord) {
    let level = morton.0 >> PARTITION;
    println!("level: {level:?}");
    println!(
        "(x: {}, y: {})",
        morton.0, morton.1,
    );
    println!(
        "(x: {}, y: {})",
        morton.0 & ((1 << PARTITION) - 1),
        morton.1 & ((1 << PARTITION) - 1),
    );
}


impl QuadTree {
    fn update_bounds(&mut self, coord: &Coord) {
        self.bounds.min_x = self.bounds.min_x.min(coord.0);
        self.bounds.min_y = self.bounds.min_y.min(coord.1);
        self.bounds.max_x = self.bounds.max_x.max(coord.0);
        self.bounds.max_y = self.bounds.max_y.max(coord.1);
    }
    fn insert_levels(&mut self, coord: &Coord) {
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
    fn get_grid(&self, coord: &Coord, level: isize) -> Option<[QuadNode; GRID]> {
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

    fn bubble_belief(&mut self, coord: &Coord, mut belief: Belief) {
        let mut homogenous = true;
        for lvl in 0..self.levels {
            if !homogenous {
                break;
            }
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
    fn cleanse_repres(&mut self, coord: &Coord) {
        let mut homogenous = false;
        let mut stack = Vec::new();
        for lvl in (1..LEVELS).rev() {
            let m_coord = encode_morton(coord, lvl);
            if let Some(n) = self.information.get(&m_coord) {
                if n.homogenous {
                    for g in child_morton(&m_coord) {
                        stack.push((lvl-1, g));
                    }
                    break;
                }
            } else {
                return;
            }
        }
        while let Some((lvl, m)) = stack.pop() {
            self.information.remove(&m);
            if lvl == 0 { continue; }
            for g in child_morton(&m) {
                stack.push((lvl - 1, g));
            }
        }
    }
    fn insert_cell(&mut self, coord: &Coord, mut belief: Belief) {
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
    fn display_with_levels(&self) {
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
            writeln!(f, "{}", line);
        }
        Ok(())
    }
}

fn main() {
    // // println!("------------------------");
    // let test = (3, 3);
    // let level = 2;
    // let m_test = encode_morton(&test, level);
    // // let test = encode_morton(&(3,3), 1);
    // // let test = encode_morton(&test, 2);
    // // for n in child_morton(&m_test) {
    // //     print_morton(&n);
    // // }
    // // for n in grid_morton(&test, level) {
    // //     print_morton(&n);
    // // }
    // let mut oracle_quad = QuadTree::new();
    // for i in 0..4 {
    //     for j in 0..4 {
    //         oracle_quad.insert_cell(&(i, j), Belief::Occupied);
    //     }
    // }
    // // oracle_quad.insert_cell(&(3, 3), Belief::Occupied);
    // println!("Oracle_quad\n{:?}", oracle_quad);
    // // println!("-------------------------------");
    // // println!("-------------------------------");
    // println!("Oracle_quad\n{}", oracle_quad);
    // println!("-------------------------------");
    // println!("-------------------------------");
    // oracle_quad.display_with_levels();

    let path = "./data/sample/test_quad0.map";
    match (readmap(path), readquad(path, LEVELS)) {
        (Ok(oracle_grid), Ok(oracle_quad)) => {
            // println!("Oracle_quad {:?}", oracle_quad.information);
            println!("-------------------------------");
            println!("Oracle Grid\n{oracle_grid}");
            println!("-------------------------------");
            println!("-------------------------------");
            println!("Oracle Quad\n{oracle_quad}");
            println!("-------------------------------");
            oracle_quad.display_with_levels();
        }
        _ => {
            println!("Unexpected Error");
        }
    }
}
