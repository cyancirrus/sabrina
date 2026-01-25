#![allow(unused)]
use crate::global::consts::GRID_OFFSET;
use crate::global::types::{Belief, Bounds, Coord};
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fmt;
use std::fs;

#[derive(Clone)]
pub struct Grid {
    pub information: HashMap<Coord, Belief>,
    pub seen: Bounds,
    pub bounds: Bounds,
}

pub trait SpatialSource<T> {
    fn get_bounds(&self) -> Bounds;
    fn get_cell(&self, coord: &Coord) -> T;
    fn set_cell(&mut self, coord: &Coord, object: T);
}
pub fn translate(xy: Coord) -> Coord {
    (
        xy.0.wrapping_add(GRID_OFFSET),
        xy.1.wrapping_add(GRID_OFFSET),
    )
}

impl Grid {
    pub fn new(information: HashMap<Coord, Belief>, bounds: Bounds) -> Self {
        let seen = Bounds {
            min_x: usize::MAX,
            min_y: usize::MAX,
            max_x: 0,
            max_y: 0,
        };
        Self {
            information,
            bounds,
            seen,
        }
    }
    pub fn path_clear(&self, xy: Coord) -> bool {
        !self.information.contains_key(&translate(xy))
    }
    pub fn raycast(&self, position: Coord, delta: Coord, max_range:usize) -> Option<Coord> {
        // mock interface owning interface don't need dynamic changing env at the moment
        // RcRefcell or ArcMutex if doing pathing with multiple as extensions
        let mut n_xy = position;
        for _ in 1..max_range {
            n_xy.0 = n_xy.0.wrapping_add(delta.0);
            n_xy.1 = n_xy.1.wrapping_add(delta.1);
            // needs to fit wrt the underlying grid
            if !self.path_clear(n_xy) {
                // denomralize b/c is oracle and needs to be relative
                let denorm_xy = (
                    n_xy.0.wrapping_sub(position.0),
                    n_xy.1.wrapping_sub(position.1),
                );
                return Some(denorm_xy);
            }
        }
        println!("--------");
        None
    }
    pub fn insert_object(&mut self, coord: Coord, obj: Belief) {
        let coord = translate(coord);
        self.seen.min_x = self.seen.min_x.min(coord.0);
        self.seen.min_y = self.seen.min_y.min(coord.1);
        self.seen.max_x = self.seen.max_x.max(coord.0);
        self.seen.max_y = self.seen.max_y.max(coord.1);
        self.bounds.min_x = self.bounds.min_x.min(coord.0);
        self.bounds.min_y = self.bounds.min_y.min(coord.1);
        self.bounds.max_x = self.bounds.max_x.max(coord.0);
        self.bounds.max_y = self.bounds.max_y.max(coord.1);
        self.information.insert(coord, obj);
    }
    pub fn get_cell(&self, xy: &Coord) -> Option<Belief> {
        let coord = translate(*xy);
        match self.information.get(&coord) {
            Some(&b) => Some(b),
            None => Some(Belief::Free),
        }
    }
}
