#![allow(unused)]

use sabrina::environment::representation::{Bounds, Environment, Object};
use sabrina::parser::map::readmap;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::error::Error;
use std::fmt;
use std::fs;

type Coord = (isize, isize);
const GRAIN: usize = 4;
// Sees in 4 principle components
pub struct Lidar {
    // Max range ould be noise informed
    pub max_range: isize,
    oracle: Environment,
}
pub struct Measurement {
    // closest objects eventually need to refactor with theta
    data: [Option<Coord>; GRAIN],
}

impl Lidar {
    pub fn new(max_range: isize, oracle: Environment) -> Self {
        Self { max_range, oracle }
    }
    fn beam(&self, position: &Coord, delta: &Coord) -> Option<Coord> {
        // Mock interface owning interface don't need dynamic changing env at the moment
        // RcRefcell or ArcMutex if doing pathing with multiple as extensions
        for h in 1..self.max_range {
            let n_xy = (position.0 + delta.0 * h, position.1 + delta.1 * h);
            if !self.oracle.path_clear(&n_xy) {
                return Some(n_xy);
            }
        }
        None
    }
    pub fn measure(&self, position: &Coord) -> Measurement {
        let mut data = [None; GRAIN];
        // Polar order of scan ie counter-clockwise
        for (h, d) in [(1, 0), (0, 1), (-1, 0), (0, -1)].iter().enumerate() {
            data[h] = self.beam(position, d);
        }
        Measurement { data }
    }
}
#[derive(Eq, PartialEq)]
struct MinNode {
    // All costs should be non negative
    cost: usize,
    coord: Coord,
}

impl MinNode {
    pub fn new(cost: usize, coord: Coord) -> Self {
        Self { cost, coord }
    }
}

impl Ord for MinNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl PartialOrd for MinNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub struct Sabrina {
    position: Coord,
    environment: Environment,
    lidar: Lidar,
}

impl Sabrina {
    pub fn new(position: Coord, environment: Environment, lidar: Lidar) -> Self {
        Self {
            position,
            environment,
            lidar,
        }
    }
    fn scan(&mut self) {
        // These will need to be rotated in order to account for orrientation
        let measure = self.lidar.measure(&self.position);
        for m in measure.data {
            if let Some((nx, ny)) = m {
                self.environment.insert_object(
                    (nx + self.position.0, ny + self.position.1),
                    Object::Unknown,
                );
            }
        }
    }
    fn estimate(source: &Coord, target: &Coord) -> usize {
        target.0.abs_diff(source.0) + target.1.abs_diff(source.1)
    }

    fn teleport(&mut self, destination: Coord) {
        self.position = destination
    }

    pub fn navigate(&mut self, target: Coord) -> bool {
        let mut p_queue: BinaryHeap<MinNode> = BinaryHeap::new();
        let mut enqueue: HashSet<Coord> = HashSet::new();
        p_queue.push(MinNode::new(
            Self::estimate(&self.position, &target),
            self.position,
        ));
        enqueue.insert(self.position);
        let neighbors = [(1, 0), (0, 1), (-1, 0), (0, -1)];

        while let Some(node) = p_queue.pop() {
            // implementing base A* before I worry about backtracking and navigation
            if node.coord == target {
                println!("!Destination achieved!");
                return true;
            } else if !self.environment.path_clear(&node.coord) {
                println!("bad state");
                // Unfortunately we're teleporting with this we sohuld like go back towards
                // unexplored but i'm not sure exactly how to do that
                continue;
            } else if !self.environment.path_clear(&target) {
                println!("unachievable target is out of bounds");
                return false
            }
            self.scan();
            // need something better here don't want to recursively navigate and i don't think i
            // can in rust
            self.teleport(node.coord);
            for (dx, dy) in neighbors {
                let nxy = (node.coord.0 + dx, node.coord.1 + dy);
                if !enqueue.contains(&nxy) && self.environment.path_clear(&nxy) {
                    enqueue.insert(nxy);
                    let cost = Self::estimate(&nxy, &target);
                    p_queue.push(MinNode::new(cost, nxy));
                }
            }
        }
        false
    }
}

fn main() {
    let path = "./data/sample/test0.map";
    match readmap(path) {
        Ok(oracle) => {
            let position = (1, 1);
            let target = (18,3);
            // println!("Test oracle pathclear 0, 1 {:?}", oracle.path_clear(&(1,2)));
            let bounds = Bounds::new(0, 0, 4, 4);
            let environment = Environment::new(HashMap::new(), bounds);
            let lidar = Lidar::new(3, oracle.clone());
            let mut sabby = Sabrina::new(position, environment, lidar);
            println!("{oracle}");

            println!("What is this {:?}", sabby.navigate(target));
        }
        Err(e) => {
            println!("Err\n{e:?}");
        }
    }
}
