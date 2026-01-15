use sabrina::environment::quad::QuadTree;
use sabrina::global::consts;
use sabrina::parser::map::readmap;
use sabrina::parser::quad::readquad;
use sabrina::environment::quad::Belief;

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
