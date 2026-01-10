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

#[derive(Eq, PartialEq, Debug)]
pub enum Status {
    Enroute,
    Blocked,
    Impossible,
    Complete,
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
                // denomralize b/c is oracle and needs to be relative
                let denorm_xy = (n_xy.0 - position.0, n_xy.1 - position.1);
                return Some(denorm_xy);
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

    fn reconstruct(precursor: &HashMap<Coord, Coord>, source: &Coord, target: &Coord) -> Vec<Coord> {
        // Ensure this is synchronized with action as this returns reversed plan
        let mut plan = vec![];
        let mut node = *target;
        while node != *source {
            plan.push(node);
            node = precursor[&node];
        }
        plan
    }

    fn plan(&self, target: Coord) -> Option<Vec<Coord>> {
        let mut p_queue: BinaryHeap<MinNode> = BinaryHeap::new();
        let mut enqueue: HashSet<Coord> = HashSet::new();
        let mut precursor = HashMap::new();

        p_queue.push(MinNode::new(
            Self::estimate(&self.position, &target),
            self.position,
        ));
        enqueue.insert(self.position);
        let neighbors = [(1, 0), (0, 1), (-1, 0), (0, -1)];

        while let Some(node) = p_queue.pop() {
            // implementing base A* before I worry about backtracking and navigation
            if node.coord == target {
                let plan = Self::reconstruct(&precursor, &self.position, &target);
                return Some(plan);
            } else if !self.environment.path_clear(&target) {
                return None;
            }
            for (dx, dy) in neighbors {
                let nxy = (node.coord.0 + dx, node.coord.1 + dy);
                if !enqueue.contains(&nxy) && self.environment.path_clear(&nxy) {
                    precursor.insert(nxy, node.coord);
                    enqueue.insert(nxy);
                    let cost = Self::estimate(&nxy, &target);
                    p_queue.push(MinNode::new(cost, nxy));
                }
            }
        }
        None
    }

    fn action(&mut self, plan: Vec<Coord>) -> Status {
        for &pos in plan.iter().rev() {
            self.scan();
            if self.environment.path_clear(&pos) {
                self.position = pos;
            } else {
                return Status::Blocked;
            }
        }
        Status::Complete
    }

    pub fn navigate(&mut self, target: Coord) -> Status {
        let mut status = Status::Enroute;
        while status != Status::Complete && status != Status::Impossible {
            let plan = self.plan(target);
            status = match plan {
                Some(p) => self.action(p),
                None => Status::Impossible,
            }
        }
        status
    }
}

fn main() {
    let path = "./data/sample/test0.map";
    match readmap(path) {
        Ok(oracle) => {
            let position = (1, 1);
            let target = (18, 3);
            // let target = (4, 2);
            // let target = (1, 2);
            // println!("Test oracle pathclear 0, 1 {:?}", oracle.path_clear(&(1,2)));
            let bounds = Bounds::new(0, 0, 4, 4);
            let environment = Environment::new(HashMap::new(), bounds);
            let lidar = Lidar::new(100, oracle.clone());
            let mut sabby = Sabrina::new(position, environment, lidar);
            println!("{oracle}");
            // sabby.scan();
            println!("What is this {:?}", sabby.navigate(target));
            println!("Final map\n{}", sabby.environment);
        }
        Err(e) => {
            println!("Err\n{e:?}");
        }
    }
}
