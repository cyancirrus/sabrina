use sabrina::algo::a_star::astar;
use sabrina::algo::d_star::{LazyPQueue, Star, dstar_lite};
use sabrina::global::consts::LEVELS;
use sabrina::parser::quad::read_quad;

#[test]
fn test_star_planners() {
    let source = (1, 1);
    let target = (18, 3);
    println!("Navigating from {source:?} -> {target:?}");
    let path = "./data/sample/test_nav0.map";
    match read_quad(path, LEVELS) {
        Ok(oracle_quad) => {
            let mut star = Star::new();
            let mut update = LazyPQueue::new();
            let astar_plan = astar(&oracle_quad, source, target);
            let dstar_plan = dstar_lite(&oracle_quad, &mut star, &mut update, source, target);
            println!("astar {astar_plan:?}");
            println!("dstar {dstar_plan:?}");
            assert!(astar_plan.len() > 0);
            assert!(dstar_plan.len() > 0);
            assert!(
                astar_plan
                    .iter()
                    .rev()
                    .zip(dstar_plan.iter())
                    .all(|(a, b)| a == b)
            );
        }
        Err(_) => {
            assert!(false, "Unexpected error in star planners");
        }
    }
}
