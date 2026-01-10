#![allow(unused)]
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fmt;
use std::fs;

type Coord = (isize, isize);
#[derive(Clone)]
pub enum Object {
    Doorway,
    Obstacle,
    Corner,
    Wall,
    Unknown,
}

#[derive(Clone, Debug)]
pub struct Bounds {
    pub min_x: isize,
    pub min_y: isize,
    pub max_x: isize,
    pub max_y: isize,
}

impl Bounds {
    pub fn new(min_x: isize, min_y: isize, max_x: isize, max_y: isize) -> Self {
        Self {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }
}
#[derive(Clone)]
pub struct Environment {
    information: HashMap<Coord, Object>,
    bounds: Bounds,
}

impl fmt::Display for Environment {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for i in (self.bounds.min_y..=self.bounds.max_y).rev() {
            let mut line = String::new();
            for j in self.bounds.min_x..=self.bounds.max_x {
                let symbol = match self.information.get(&(j, i)) {
                    // None => '\u{00b7}',
                    None => ' ',
                    Some(Object::Corner) => 'x',
                    Some(Object::Doorway) => '+',
                    Some(Object::Obstacle) => '*',
                    Some(Object::Unknown) => '?',
                    Some(Object::Wall) => '#',
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

impl Environment {
    pub fn new(information: HashMap<Coord, Object>, bounds: Bounds) -> Self {
        Self {
            information,
            bounds,
        }
    }
    pub fn path_clear(&self, xy: &Coord) -> bool {
        !self.information.contains_key(xy)
    }
    pub fn insert_object(&mut self, coord: Coord, obj: Object) {
        self.bounds.min_x = self.bounds.min_x.min(coord.0);
        self.bounds.min_y = self.bounds.min_y.min(coord.1);
        self.bounds.max_x = self.bounds.max_x.max(coord.0);
        self.bounds.max_y = self.bounds.max_y.max(coord.1);
        self.information.insert(coord, obj);
    }
}
