use crate::environment::grid::Grid;
use crate::global::types::{Belief, Bounds};
use std::collections::HashMap;
use std::error::Error;
use std::fs;

pub fn read_grid(path: &str) -> Result<Grid, Box<dyn Error>> {
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
                b'?' => Belief::Unknown,
                b'#' => Belief::Occupied,
                b'+' => Belief::Occupied,
                b'*' => Belief::Occupied,
                b'x' => Belief::Occupied,
                _ => {
                    return Err(
                        format!("Unexpected symbol found in map with source {path:?}").into(),
                    );
                }
            };
            mirrored_objects.push(((idx_x as isize, idx_y as isize), obj));
            max_x = max_x.max(idx_x as isize);
        }
        max_y = max_y.max(idx_y as isize);
    }
    let mut information = HashMap::new();
    let bounds = Bounds::new(0, 0, max_x, max_y);
    // Mapping is easiest to think of as direct representation ie mirrored b/c of parsing
    for ((idx_x, mir_idx_y), obj) in mirrored_objects {
        information.insert((idx_x, max_y - mir_idx_y), obj);
    }

    Ok(Grid::new(information, bounds))
}
