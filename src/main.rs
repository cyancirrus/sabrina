#![allow(unused)]
use sabrina::environment::quad::QuadTree;
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::global::consts::LEVELS;
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
    // let partition = 4;
    // let level = (morton.0 >> partition) - 1;
    let partition = 4;
    let level = 1;
    let x1:u8 = 0b_1100 | 1 << partition;
    let x2:u8 = 0b_1110 | 1 << partition;
   
    println!("x1: {x1:b}");
    println!("x2: {x2:b}");

    let y1 = x1 >> partition;
    println!("y1: {y1:}");
    // let level = (morton.0 >> PARTITION) - 1;

    // for lvl in 0..8 {
    //     println!("lvl {lvl:?}");
    //     let mask:u8 = !((1 << lvl) - 1);
    //     println!("mask {lvl:}: {mask:b}");
    // }




    // let mut oracle_quad = QuadTree::new();
    // for i in 0..4 {
    //     for j in 0..4 {
    //         oracle_quad.insert_cell(&(i, j), Belief::Occupied);
    //     }
    // }
    // println!("Oracle_quad\n{:?}", oracle_quad);
    // println!("-------------------------------");
    // println!("-------------------------------");
    // println!("Oracle_quad\n{}", oracle_quad);
    // println!("-------------------------------");
    // println!("-------------------------------");
    // oracle_quad.display_with_levels();

    let path = "./data/sample/test_quad1.map";
    match (readmap(path), readquad(path, LEVELS)) {
        (Ok(oracle_grid), Ok(oracle_quad)) => {
            // println!("Oracle_quad {:?}", oracle_quad.information);
            println!("-------------------------------");
            println!("Oracle Grid\n{oracle_grid}");
            println!("-------------------------------");
            println!("-------------------------------");
            println!("Oracle Quad\n{oracle_quad}");
            println!("-------------------------------");
            println!("Oracle Quad\n{oracle_quad:?}");
            println!("-------------------------------");
            oracle_quad.display_with_levels();
        }
        _ => {
            println!("Unexpected Error");
        }
    }
}
