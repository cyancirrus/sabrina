use sabrina::algo::a_star::AStarPlanner;
use sabrina::algo::d_star::DStarPlanner;
use sabrina::global::types::ACoord;
use sabrina::global::types::PlanIter;
use sabrina::global::types::plan::Planner;
use sabrina::parser::grid::read_grid;
// use sabrina::algo::best_first::BestFirstPlanner;

#[test]
fn test_star_planners() {
    let path = "./data/sample/test_nav0.map";
    match read_grid(path) {
        Ok(oracle) => {
            let source = ACoord { x: 1, y: 1 };
            let target = ACoord { x: 18, y: 3 };
            println!("Navigating from {source:?} -> {target:?}");
            let mut dstar_planner = DStarPlanner::new();
            let mut astar_planner = AStarPlanner {};

            let astar_plan = astar_planner.plan(&oracle, source, target).unwrap();
            let dstar_plan = dstar_planner.plan(&oracle, source, target).unwrap();
            println!("astar {astar_plan:?}");
            println!("dstar {dstar_plan:?}");
            assert!(astar_plan.nodes().len() > 0);
            assert!(dstar_plan.nodes().len() > 0);
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
