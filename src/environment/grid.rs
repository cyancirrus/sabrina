#![allow(unused)]
use crate::global::types::{Bounds, Coord, Belief};
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fmt;
use std::fs;

#[derive(Clone)]
pub struct Grid {
    pub information: HashMap<Coord, Belief>,
    pub bounds: Bounds,
}

pub trait SpatialSource<T> {
    fn get_bounds(&self) -> Bounds;
    fn get_cell(&self, coord: &Coord) -> T;
    fn set_sell(&mut self, coord: &Coord, object: T);
}

impl Grid {
    pub fn new(information: HashMap<Coord, Belief>, bounds: Bounds) -> Self {
        Self {
            information,
            bounds,
        }
    }
    pub fn path_clear(&self, xy: &Coord) -> bool {
        !self.information.contains_key(xy)
    }
    pub fn insert_object(&mut self, coord: Coord, obj: Belief) {
        self.bounds.min_x = self.bounds.min_x.min(coord.0);
        self.bounds.min_y = self.bounds.min_y.min(coord.1);
        self.bounds.max_x = self.bounds.max_x.max(coord.0);
        self.bounds.max_y = self.bounds.max_y.max(coord.1);
        self.information.insert(coord, obj);
    }
}
