use crate::environment::grid::Grid;
use crate::global::types::{Belief, MinNode, Coord};
use crate::sensor::lidar::{Lidar, Status};
use std::collections::{BinaryHeap, HashMap, HashSet};
use crate::environment::info::reconstruct;

pub struct Sabrina {
    pub position: Coord,
    pub environment: Grid,
    lidar: Lidar,
}

impl Sabrina {
    pub fn new(position: Coord, environment: Grid, lidar: Lidar) -> Self {
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
                    Belief::Unknown,
                );
            }
        }
    }
    fn estimate(source: &Coord, target: &Coord) -> usize {
        target.0.abs_diff(source.0) + target.1.abs_diff(source.1)
    }

    pub fn plan(&self, target: Coord) -> Option<Vec<Coord>> {
        if !self.environment.path_clear(&target) {
            return None;
        };
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
            if node.coord == target {
                let plan = reconstruct(&precursor, &self.position, &target);
                return Some(plan);
            }
            for (dx, dy) in neighbors {
                let nxy = (node.coord.0 + dx, node.coord.1 + dy);
                if !enqueue.contains(&nxy) && self.environment.path_clear(&nxy) {
                    precursor.insert(nxy, node.coord);
                    enqueue.insert(nxy);
                    let cost = node.cost + Self::estimate(&nxy, &target);
                    p_queue.push(MinNode::new(cost, nxy));
                }
            }
        }
        None
    }

    pub fn action(&mut self, plan: Vec<Coord>) -> Status {
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
            println!("{}", self.environment);
            println!("-------------------------------");
            let plan = self.plan(target);
            status = match plan {
                Some(p) => self.action(p),
                None => Status::Impossible,
            }
        }
        status
    }
}
