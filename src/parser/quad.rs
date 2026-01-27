use crate::environment::quad::QuadTree;
use crate::global::types::{Belief, Bounds};
use std::collections::HashMap;
use std::error::Error;
use std::fs;

/// Reads the quadrant has to mirror around the y axis to align with intuition
pub fn read_quad(path: &str, levels: usize) -> Result<QuadTree, Box<dyn Error>> {
    let content = match fs::read_to_string(path) {
        Ok(c) => c,
        Err(e) => return Err(format!("Unable to read path {path:?}\n{e:?}").into()),
    };
    let (mut max_x, mut max_y) = (0, 0);
    let mut mirrored_objects = Vec::new();
    for (idx_y, line) in content.lines().enumerate() {
        for (idx_x, cell) in line.as_bytes().chunks_exact(3).enumerate() {
            let obj = match cell[1] {
                b' ' => Belief::Free,
                b'+' => Belief::Occupied,
                b'*' => Belief::Occupied,
                b'#' => Belief::Occupied,
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
    let information = HashMap::new();
    let seen = Bounds::new(0, 0, max_x, max_y);

    let padding = Bounds ::new(
        0,
        0,
        max_x + (1 << (levels - 1)),
        max_y + (1 << (levels - 1)),
    );

    // let mut quadtree = QuadTree::initialize(information, bounds, levels);
    let mut quadtree = QuadTree {
        information,
        padding,
        seen,
        levels,
    };

    // Mapping is easiest to think of as direct representation ie mirrored b/c of parsing
    for ((idx_x, mir_idx_y), obj) in mirrored_objects {
        quadtree.insert_cell(&(idx_x, max_y - mir_idx_y), obj);
    }

    Ok(quadtree)
}
