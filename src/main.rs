#![allow(unused)]
use sabrina::environment::morton::{encode_morton, grid_morton};
use sabrina::environment::quad::QuadTree;
use sabrina::global::consts;
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::map::readmap;
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
    let mut quad = QuadTree::new();
    quad.display_with_levels();
    println!("quad bounds {:?}", quad.bounds);
    println!("quad bounds {:?}", quad);

    println!("-------------------------------");
    println!("------ BASE PARSE -------------");
    println!("-------------------------------");
    let path = "./data/sample/test_quad1.map";
    match (readmap(path), readquad(path, consts::LEVELS)) {
        (Ok(oracle_grid), Ok(oracle_quad)) => {
            // println!("Oracle_quad {:?}", oracle_quad.information);
            // println!("-------------------------------");
            // println!("Oracle Grid\n{oracle_grid}");
            // println!("-------------------------------");
            // println!("-------------------------------");
            // println!("Oracle Quad\n{oracle_quad}");
            // println!("-------------------------------");
            println!("Oracle Quad\n{oracle_quad:?}");
            println!("-------------------------------");
            oracle_quad.display_with_levels();
        }
        _ => {
            println!("Unexpected Error");
        }
    }
}
