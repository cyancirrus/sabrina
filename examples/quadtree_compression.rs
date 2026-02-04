use sabrina::environment::quad::QuadTree;
use sabrina::global::consts;
use sabrina::global::types::{ACoord, Belief};
use sabrina::parser::grid::read_grid;
use sabrina::parser::quad::read_quad;

fn main() {
    let mut quad = QuadTree::new();
    println!("{quad:?}");
    println!("--------------------------------");
    quad.update_belief(&ACoord { x: 1, y: 1 }, Belief::Occupied);
    println!("{quad:?}");
    println!("----------------------------------");
    println!("{quad}");
    println!("----------------------------------");
    quad.update_belief(&ACoord { x: 0, y: 0 }, Belief::Occupied);
    quad.update_belief(&ACoord { x: 0, y: 1 }, Belief::Occupied);
    quad.update_belief(&ACoord { x: 1, y: 0 }, Belief::Occupied);
    println!("----------------------------------");
    println!("{quad:?}");

    let path = "./data/sample/test_nav0.map";
    match (read_grid(path), read_quad(path, consts::LEVELS)) {
        (Ok(_oracle_grid), Ok(oracle_quad)) => {
            println!("Oracle Quad\n{oracle_quad}");
            println!("-------------------------------");
            println!("Oracle Quad nodes\n{:?}", oracle_quad.information.len());
            println!("-------------------------------");
            println!("Oracle Quad\n{oracle_quad:?}");
        }
        _ => {
            println!("Unexpected Error");
        }
    }
}
