use sabrina::algo::d_star::{Star, dstar_lite};
use sabrina::algo::static_astar::astar;
use sabrina::global::consts::LEVELS;
use sabrina::global::types::LazyPQueue;
use sabrina::global::types::PlanIter;
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
            assert!(astar_plan.plan.len() > 0);
            assert!(dstar_plan.plan.len() > 0);
            assert!(
                astar_plan
                    .iter()
                    .zip(dstar_plan.iter())
                    .all(|(a, d)| a == d),
            );
        }
        Err(_) => {
            assert!(false, "Unexpected error in star planners");
        }
    }
}
