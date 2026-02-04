#![allow(unused)]
use sabrina::algo::a_star::AStarPlanner;
use sabrina::algo::best_first::BestFirstPlanner;
use sabrina::algo::d_star::DStarPlanner;
use sabrina::environment::grid::Grid;
use sabrina::global::types::{ACoord, HCoord};
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::environment::quad::QuadTree;
use sabrina::parser::quad::read_quad;
use sabrina::parser::grid::read_grid;
use sabrina::sensor::lidar::Lidar;
use sabrina::global::types::{SpatialMap};
use sabrina::global::types::Belief;

// TODO: We need to update the vertex for all of the subgrids

fn main() {
    println!("------------------------------------");
    println!("      Example navigation            ");
    println!("------------------------------------");
    let path = "./data/sample/test_nav0.map";
    // let path = "./data/sample/test_nav1.map";
    let levels = 1;
    match (read_quad(path, levels), read_grid(path)) {
        (Ok(q_oracle), Ok(g_oracle)) => {
            // works with levels = 5 for d*lite
            let position = ACoord { x: 1, y: 1 };
            let target = ACoord { x: 1, y: 5 };
            
            let position = ACoord { x: 1, y: 1 };
            let target = ACoord { x: 11, y: 3 };
            // works with levels = 2 for d*lite
            // let position = ACoord { x: 1, y: 1 };
            // let target = ACoord { x: 18, y: 3 };
            let environment = QuadTree::init(levels);
            // let environment = Grid::new();
            let lidar = Lidar::new(12, g_oracle.clone());
            // let mut sabby = Sabrina::new(position, environment, lidar, BestFirstPlanner);
            // let mut sabby = Sabrina::new(position, environment, lidar, AStarPlanner);
            let mut sabby = Sabrina::new(position, environment, lidar, DStarPlanner::new());
            println!("absolute_environment\n{q_oracle}");
            // println!("-------------------------------");
            // println!("    Starting Navigation        ");
            // println!("-------------------------------");
            println!("Final Status {:?}", sabby.navigate(target));
            println!("Final map\n{}", sabby.environment);
        }
        _ => {
            println!("Err");
        }
    }
}
