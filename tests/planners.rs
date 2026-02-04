use sabrina::algo::a_star::AStarPlanner;
use sabrina::algo::d_star::DStarPlanner;
// use sabrina::algo::best_first::BestFirstPlanner;

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
            let position = (1, 1);
            let target = (18, 3);
            let environment = Grid::new();
            let lidar = Lidar::new(12, oracle.clone());
            let mut sabby = Sabrina::new(position, environment.clone(), lidar.clone(), DStarPlanner::new());
            let dstar_plan = sabby.planner.plan(&sabby.environment, sabby.position, target);
            sabby.planner = AStarPlanner;
            let astar_plan = sabby.planner.plan(&sabby.environment, sabby.position, target);

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
