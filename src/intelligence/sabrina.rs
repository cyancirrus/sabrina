use crate::global::types::plan::PlanIter;
use crate::global::types::{Coord, Status};
use crate::global::types::{Planner, SpatialMap};
use crate::sensor::lidar::Lidar;
use std::fmt::{Debug, Display};

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
    P::Plan: Debug,
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
                let obstacle = (
                    nx.wrapping_add(self.position.0),
                    ny.wrapping_add(self.position.1),
                );
                self.environment.insert_ray(self.position, obstacle);
                // should check and only replan if new info
                self.planner
                    .update(&self.environment, self.position, obstacle);
            }
        }
    }
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
        // self.scan();
        let mut status = Status::Enroute;
        while status != Status::Complete && status != Status::Impossible {
            println!("Environment\n{}", self.environment);
            println!("-------------------------------");
            let plan = self.planner.plan(&self.environment, self.position, target);
            if plan.is_some() {
                println!("some plan");
            } else {
                println!("no plan");
            }
            status = match plan {
                Some(p) => self.action(p),
                None => Status::Impossible,
            }
        }
        status
    }
}
