use crate::global::types::plan::PlanIter;
use crate::global::types::{ACoord, Status};
use crate::global::types::{Planner, SpatialMap};
use crate::sensor::lidar::Lidar;
use std::fmt::{Debug, Display};

pub struct Sabrina<S, P>
where
    S: SpatialMap,
    P: Planner<S>,
{
    pub environment: S,
    pub lidar: Lidar,
    pub planner: P,
    pub position: ACoord,
}

impl<S, P> Sabrina<S, P>
where
    S: SpatialMap + Display,
    P: Planner<S>,
    P::Plan: Debug,
{
    pub fn new(position: ACoord, environment: S, lidar: Lidar, planner: P) -> Self {
        Self {
            environment,
            lidar,
            planner,
            position,
        }
    }
    fn scan(&mut self) {
        // These will need to be rotated in order to account for orrientation
        let measure = self.lidar.measure(self.position);
        for m in measure.data {
            if let Some(n) = m {
                let obstacle = ACoord {
                    x: n.x + self.position.x,
                    y: n.y + self.position.y,
                };
                self.planner
                    .update(&self.environment, self.position, obstacle);
                self.environment.insert_ray(self.position, obstacle);
                // should check and only replan if new info
            }
        }
    }
    pub fn action<Q: PlanIter>(&mut self, plan: Q) -> Status {
        for &tgt in plan.iter() {
            let status = self.control(tgt);
            if status == Status::Blocked {
                return Status::Blocked 
            }
        }
        Status::Complete
    }
    pub fn navigate(&mut self, target: ACoord) -> Status {
        self.environment.initialize(self.position, target);
        self.scan();
        let mut status = Status::Enroute;
        while status != Status::Complete && status != Status::Impossible {
            println!("Environment\n{}", self.environment);
            println!("-------------------------------");
            let plan = self.planner.plan(&self.environment, self.position, target);
            status = match plan {
                Some(p) => self.action(p),
                None => Status::Impossible,
            }
        }
        status
    }
    pub fn control(&mut self, tgt: ACoord) -> Status
    {
        // beliefs not recorded are assumed unknown
        // handles simulation compass rose signals
        let mut pos = self.position;
        let (dy, dx) = (tgt.y - pos.y, tgt.x - pos.x);
        let (del_y, del_x) = (dy.signum(), dx.signum());
        println!("env {:}", self.environment);
        while !self.environment.obstructed(pos) {
            if tgt == self.position {
                return Status::Enroute
            }
            self.position = pos;
            self.scan();
            println!("NEW POS {pos:?}");
            pos.x += del_x;
            pos.y += del_y;
        }
        Status::Blocked
    }
    // pub fn action<Q: PlanIter>(&mut self, plan: Q) -> Status {
    //     for &pos in plan.iter() {
    //         self.scan();
    //         if !self.environment.obstructed(pos) {
    //             println!("new pos {pos:?}");
    //             self.position = pos;
    //         } else {
    //             return Status::Blocked;
    //         }
    //     }
    //     Status::Complete
    // }
    // pub fn navigate(&mut self, target: ACoord) -> Status {
    //     self.environment.initialize(self.position, target);
    //     self.scan();
    //     let mut status = Status::Enroute;
    //     while status != Status::Complete && status != Status::Impossible {
    //         println!("Environment\n{}", self.environment);
    //         println!("-------------------------------");
    //         let plan = self.planner.plan(&self.environment, self.position, target);
    //         status = match plan {
    //             Some(p) => self.action(p),
    //             None => Status::Impossible,
    //         }
    //     }
    //     status
    // }
}
