#![allow(unused)]
use std::collections::{HashSet,HashMap};
use std::fmt;
use std::fs;
use std::error::Error;

type Coord = (isize, isize);
pub enum Object {
    Doorway,
    Obstacle,
    Corner,
    Wall,
}

pub struct Bounds {
    min_x: isize,
    min_y: isize,
    max_x: isize,
    max_y: isize,
}



impl Bounds {
    pub fn new(min_x:isize, min_y:isize, max_x:isize, max_y:isize) -> Self {
        Self { min_x, min_y, max_x, max_y }
    }
}

pub struct Environment {
    information: HashMap<Coord, Object>,
    bounds: Bounds,
}

impl fmt::Display for Environment {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for i in (self.bounds.min_y..self.bounds.max_y).rev() {
            let mut line = String::new();
            for j in self.bounds.min_x..self.bounds.max_x {
                let symbol = match self.information.get(&(j,i)) {
                    // None => '\u{00b7}',
                    None => ' ',
                    Some(Object::Doorway)  => '+',
                    Some(Object::Obstacle) => '*',
                    Some(Object::Wall)  => '#',
                    Some(Object::Corner) => 'x',
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
    pub fn new(information: HashMap<Coord, Object>, bounds:Bounds) -> Self {
        Self { information , bounds}
    }
    pub fn path_clear(&self, xy:&Coord) -> bool {
        !self.information.contains_key(xy)
    }
}

