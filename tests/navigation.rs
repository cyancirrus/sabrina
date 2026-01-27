use sabrina::environment::grid::Grid;
use sabrina::global::types::{Bounds, Status};
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::read_grid;
use sabrina::sensor::lidar::Lidar;
use std::collections::HashMap;


#[test]
fn test_navigation() {
    let path = "./data/sample/test_nav0.map";
    match read_grid(path) {
        Ok(oracle) => {
            let position = (1, 1);
            let target = (18, 3);
            let environment = Grid::new();
            let lidar = Lidar::new(100, oracle.clone());
            let mut sabby = Sabrina::new(position, environment, lidar);
            assert_eq!(Status::Complete, sabby.navigate(target));
        }
        Err(e) => {
            assert!(false, "Error has occured in the navigation");
        }
    }
}
