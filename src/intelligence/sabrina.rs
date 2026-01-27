use crate::global::types::plan::PlanIter;
use crate::global::types::{Coord, Status};
use crate::global::types::{Planner, SpatialMap};
use crate::sensor::lidar::Lidar;
use std::fmt::Display;

pub struct Sabrina<S, P>
where
    S: SpatialMap,
    P: Planner<S>,
{
    pub position: Coord,
    pub environment: S,
    lidar: Lidar,
    planner: P,
}

impl<S, P> Sabrina<S, P>
where
    S: SpatialMap + Display,
    P: Planner<S>,
    // <P Plan::Planner<S>>::Plan>
{
    pub fn new(position: Coord, environment: S, lidar: Lidar, planner: P) -> Self {
        Self {
            position,
            planner,
            environment,
            lidar,
        }
    }
    fn scan(&mut self) {
        // These will need to be rotated in order to account for orrientation
        let measure = self.lidar.measure(self.position);
        for m in measure.data {
            if let Some((nx, ny)) = m {
                self.environment.insert_ray(
                    self.position,
                    (
                        nx.wrapping_add(self.position.0),
                        ny.wrapping_add(self.position.1),
                    ),
                );
            }
        }
    }
    // pub fn best_first_plan(&self, target: Coord) -> Option<Vec<Coord>> {
    //     if self.environment.obstructed(target) {
    //         return None;
    //     };
    //     let mut p_queue: MinHeap<Coord> = MinHeap::new();
    //     let mut enqueue: HashSet<Coord> = HashSet::new();
    //     let mut precursor = HashMap::new();
    //     p_queue.push(MinNode::new(0, self.position));
    //     enqueue.insert(self.position);
    //     let neighbors = [(1, 0), (0, 1), (!0, 0), (0, !0)];
    //     while let Some(node) = p_queue.pop() {
    //         if node.coord == target {
    //             let plan = reconstruct(&precursor, self.position, target);
    //             return Some(plan);
    //         }
    //         for (dx, dy) in neighbors {
    //             let n_xy = (node.coord.0.wrapping_add(dx), node.coord.1.wrapping_add(dy));
    //             if !enqueue.contains(&n_xy) && !self.environment.obstructed(n_xy) {
    //                 precursor.insert(n_xy, node.coord);
    //                 enqueue.insert(n_xy);
    //                 let cost = self.environment.distance(
    //                     self.environment.encode(n_xy),
    //                     self.environment.encode(target),
    //                 );
    //                 p_queue.push(MinNode::new(cost, n_xy));
    //             }
    //         }
    //     }
    //     None
    // }
    pub fn action<Q: PlanIter>(&mut self, plan: Q) -> Status {
        for &pos in plan.iter() {
            self.scan();
            if !self.environment.obstructed(pos) {
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
            let plan = self.planner.plan(&self.environment, self.position, target);
            status = match plan {
                Some(p) => self.action(p),
                None => Status::Impossible,
            }
        }
        status
    }
}
