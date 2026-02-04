use sabrina::environment::new_quad::QuadTree;
use sabrina::global::consts;
use sabrina::global::types::{ACoord, Belief};
use sabrina::parser::grid::read_grid;
use sabrina::parser::new_quad::read_quad;

fn main() {
    let mut quad = QuadTree::new();
    // quad.display_with_levels();
    println!("--------------------------------");
    quad.update_belief(&ACoord { x: 1, y: 1 }, Belief::Occupied);
    quad.display_with_levels();
    println!("----------------------------------");
    println!("{quad}");
    println!("----------------------------------");
    quad.update_belief(&ACoord { x: 0, y: 0 }, Belief::Occupied);
    quad.update_belief(&ACoord { x: 0, y: 1 }, Belief::Occupied);
    quad.update_belief(&ACoord { x: 1, y: 0 }, Belief::Occupied);
    println!("----------------------------------");
    quad.display_with_levels();

    let path = "./data/sample/test_nav0.map";
    match (read_grid(path), read_quad(path, consts::LEVELS)) {
        (Ok(_oracle_grid), Ok(oracle_quad)) => {
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
