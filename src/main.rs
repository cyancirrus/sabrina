#![allow(unused)]

use std::collections::{HashSet,HashMap};
use std::fmt;
use std::fs;
use std::error::Error;

type Coord = (isize, isize);
enum Object {
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

fn readmap(path:&str) -> Result<Environment, Box<dyn Error>> {
    let content = match fs::read_to_string(path) {
        Ok(c) => c,
        Err(e) => return Err(format!("Unable to read path {path:?}\n{e:?}").into()),
    };
    let (mut max_x, mut max_y) = (0, 0);
    let mut mirrored_objects = Vec::new();
    for (idx_y, line) in content.lines().enumerate() {
        for  (idx_x, cell) in line.as_bytes().chunks_exact(3).enumerate() {
            let obj = match cell[1] {
                b' ' => continue,
                b'+' => Object::Doorway,
                b'*' => Object::Obstacle,
                b'#' => Object::Wall,
                b'x' => Object::Corner,
                _ => return Err(format!("Unexpected symbol found in map with source {path:?}").into()),
            };
            mirrored_objects.push(((idx_x as isize, idx_y as isize), obj));
            max_x = max_x.max(idx_x as isize);
        }
        max_y = max_y.max(idx_y as isize);
    }
    let mut information = HashMap::new();
    let bounds = Bounds::new(0, 0, max_x, max_y);
    // Mapping is easiest to think of as direct representation ie mirrored b/c of parsing
    for ((mir_idx_x, mir_idx_y), obj) in mirrored_objects {
        information.insert( (max_x - mir_idx_x , max_y - mir_idx_y), obj);
    }

    Ok(Environment { information, bounds })
}


impl Bounds {
    fn new(min_x:isize, min_y:isize, max_x:isize, max_y:isize) -> Self {
        Self { min_x, min_y, max_x, max_y }
    }
}

struct Environment {
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

fn samplepath() -> Environment {
    let mut information = HashMap::new();
    let bounds = Bounds::new(0, 0, 8, 4);
    let environment = vec![
        ((1,0), Object::Obstacle),
        ((1,1), Object::Obstacle),
        ((3,3), Object::Obstacle),
        ((3,2), Object::Obstacle),
        ((3,1), Object::Obstacle),
        
        ((5,0), Object::Wall),
        ((5,1), Object::Wall),
        ((5,2), Object::Wall),
        ((5,3), Object::Doorway),
    ];
    for (coord, object) in environment {
        information.insert(coord, object);
    }
    Environment { information, bounds }
}

fn main() {
    let mut sample = samplepath();
    println!("{sample:}");
}
