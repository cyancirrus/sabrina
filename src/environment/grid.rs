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
    bounds: Bounds,
}

pub trait SpatialSource<T> {
    fn get_bounds(&self) -> Bounds;
    fn get_cell(&self, coord: &Coord) -> T;
    fn set_cell(&mut self, coord: &Coord, object: T);
}
pub fn translate(xy: Coord) -> Coord {
    (xy.0 + GRID_OFFSET, xy.1 + GRID_OFFSET)
}

impl Grid {
    pub fn new(information: HashMap<Coord, Belief>, bounds: Bounds) -> Self {
        let seen = Bounds {
            min_x:usize::MAX, min_y:usize::MAX, max_x:0, max_y:0
        };
        Self {
            information,
            bounds,
            seen,
        }
    }
    pub fn path_clear(&self, xy: Coord) -> bool {
        !self.information.contains_key(
            &translate(xy)
        )
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
        match self.information.get(xy) {
            Some(&b) => Some(b),
            None => Some(Belief::Free),
        }
    }
}
