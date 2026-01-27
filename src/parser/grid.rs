use crate::environment::grid::Grid;
use crate::global::consts::GRID_OFFSET;
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
            mirrored_objects.push(((idx_x, idx_y), obj));
            max_x = max_x.max(idx_x);
        }
        max_y = max_y.max(idx_y);
    }
    let mut information = HashMap::new();
    // Mapping is easiest to think of as direct representation ie mirrored b/c of parsing
    for ((idx_x, mir_idx_y), obj) in mirrored_objects {
        let coord = (idx_x + GRID_OFFSET, max_y - mir_idx_y + GRID_OFFSET);
        information.insert(coord, obj);
    }
    let bounds = Bounds {
        min_x: 0,
        min_y: 0,
        max_x: max_x,
        max_y: max_y,
    };
    let seen = Bounds {
        min_x: GRID_OFFSET,
        min_y: GRID_OFFSET,
        max_x: max_x + GRID_OFFSET,
        max_y: max_y + GRID_OFFSET,
    };

    Ok(Grid {
        information,
        seen,
        bounds,
    })
}
