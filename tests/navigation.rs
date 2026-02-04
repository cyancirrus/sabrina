use sabrina::algo::best_first::BestFirstPlanner;
use sabrina::global::types::ACoord;
use sabrina::environment::grid::Grid;
use sabrina::global::types::Status;
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::read_grid;
use sabrina::sensor::lidar::Lidar;

#[test]
fn test_navigation() {
    let path = "./data/sample/test_nav0.map";
    match read_grid(path) {
        Ok(oracle) => {
            let source = ACoord { x: 1, y: 1 };
            let target = ACoord { x: 18, y: 3 };
            let environment = Grid::new();
            let lidar = Lidar::new(100, oracle.clone());
            let mut sabby = Sabrina::new(source, environment, lidar, BestFirstPlanner);
            assert_eq!(Status::Complete, sabby.navigate(target));
        }
        Err(_) => {
            assert!(false, "Error has occured in the navigation");
        }
    }
}
