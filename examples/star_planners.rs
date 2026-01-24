use sabrina::algo::a_star::astar;
use sabrina::algo::d_star::{LazyPQueue, Star, dstar_lite};
use sabrina::global::consts::LEVELS;
use sabrina::parser::quad::read_quad;
use std::time::Instant;

fn main() {
    let source = (1, 1);
    let target = (18, 3);
    println!("Navigating from {source:?} -> {target:?}");

    let path = "./data/sample/test_nav0.map";
    // let path = "./data/sample/test_quad0.map";
    match read_quad(path, LEVELS) {
        Ok(oracle_quad) => {
            println!("Oracle Quad\n{oracle_quad}");
            println!("-------------------------------");
            oracle_quad.display_with_levels();
            println!("-------------------------------");
            let mut star = Star::new();
            let mut update = LazyPQueue::new();
            let start = Instant::now();
            // let plan = astar(&oracle_quad, source, target);
            let plan = dstar_lite(&oracle_quad, &mut star, &mut update, source, target);
            println!("plan {plan:?}");
            println!("Duration D*Lite {:?}", start.elapsed());
            let start = Instant::now();
            let _plan = astar(&oracle_quad, source, target);
            println!("Duration A* {:?}", start.elapsed());
            println!("------------------------------");
            println!("------------------------------");
        }
        _ => {
            println!("Unexpected Error");
        }
    }
}
