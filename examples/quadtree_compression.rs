#![allow(unused)]
use sabrina::environment::quad::QuadTree;
use sabrina::global::consts::LEVELS;
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::map::readmap;
use sabrina::parser::quad::readquad;
use sabrina::sensor::lidar::Lidar;
use std::collections::HashMap;

fn main() {
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
