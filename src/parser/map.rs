use std::collections::HashMap;
use std::fs;
use std::error::Error;
use crate::environment::representation::{Environment, Bounds, Object};

pub fn readmap(path:&str) -> Result<Environment, Box<dyn Error>> {
    let content = match fs::read_to_string(path) {
        Ok(c) => c,
        Err(e) => return Err(format!("Unable to read path {path:?}\n{e:?}").into()),
    };
    let (mut max_x, mut max_y) = (0, 0);
    let mut mirrored_objects = Vec::new();
    for (idx_y, line) in content.lines().enumerate() {
        for (idx_x, cell) in line.as_bytes().chunks_exact(3).enumerate() {
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
    let bounds = Bounds::new(0, 0, max_x + 1, max_y + 1);
    // Mapping is easiest to think of as direct representation ie mirrored b/c of parsing
    for ((idx_x, mir_idx_y), obj) in mirrored_objects {
        information.insert( (idx_x , max_y - mir_idx_y), obj);
    }

    Ok(Environment::new(information, bounds ))
}
