#![allow(unused)]
use sabrina::environment::morton::{child_morton, encode_morton, grid_morton, print_morton};
use sabrina::environment::quad::QuadTree;
use sabrina::global::consts;
use sabrina::global::types::Belief;
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::readmap;
use sabrina::parser::quad::readquad;
use sabrina::sensor::lidar::Lidar;
use std::collections::HashMap;

// // TODO: Bit-level Quadtree Fixes (Off-by-one/Masks)
// // Unknown State Initialization + Visual Debugger (Essential for visibility)
// // Observation Logic (Unknown \(\rightarrow \) Free/Occupied).LU Pivoting (Numerical insurance)
// // Multi-ray LiDAR & Planner Implementation.
// // Hestereses or defered clean up
// // Consdier implementing a jump iter

fn main() {
    // let mut quad = QuadTree::new(8, 4, 4);
    let mut quad = QuadTree::new(4, 4, 3);
    quad.display_with_levels();
    println!("----------------------------------");
    quad.insert_cell(&(1, 1), Belief::Occupied);
    quad.display_with_levels();
    println!("----------------------------------");
    println!("{quad}");
    println!("----------------------------------");
    quad.insert_cell(&(0, 0), Belief::Occupied);
    quad.insert_cell(&(0, 1), Belief::Occupied);
    quad.insert_cell(&(1, 0), Belief::Occupied);
    println!("----------------------------------");
    quad.display_with_levels();

    let path = "./data/sample/test_quad0.map";
    match (readmap(path), readquad(path, consts::LEVELS)) {
        (Ok(oracle_grid), Ok(oracle_quad)) => {
            println!("Oracle Quad\n{oracle_quad}");
            println!("-------------------------------");
            println!("Oracle Quad nodes\n{:?}", oracle_quad.information.len());
            println!("-------------------------------");
            oracle_quad.display_with_levels();
        }
        _ => {
            println!("Unexpected Error");
        }
    }
}
