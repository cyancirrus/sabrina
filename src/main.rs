#![allow(unused)]
use sabrina::algo::a_star::AStarPlanner;
use sabrina::algo::best_first::BestFirstPlanner;
use sabrina::algo::d_star::DStarPlanner;
use sabrina::environment::grid::Grid;
use sabrina::environment::quad::QuadTree;
use sabrina::global::types::Belief;
use sabrina::global::types::SpatialMap;
use sabrina::global::types::{ACoord, HCoord};
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::read_grid;
use sabrina::parser::quad::read_quad;
use sabrina::sensor::lidar::Lidar;
// use sabrina::hierarchy::proximity:: grid_siblings;

// TODO: We need to update the vertex for all of the subgrids
// TODO: think i need to call the update for the grid neighbors as well as edge neighbors

fn main() {
    // println!("------------------------------------");
    // println!("      Example navigation            ");
    // println!("------------------------------------");
    // let path = "./data/sample/test_nav0.map";
    // // let path = "./data/sample/test_nav1.map";
    // let levels = 3;
    // match (read_quad(path, levels), read_grid(path)) {
    //     (Ok(q_oracle), Ok(g_oracle)) => {
    //         // works with levels = 5 for d*lite
    //         let position = ACoord { x: 1, y: 1 };
    //         let target = ACoord { x: 1, y: 5 };
            
    //         // // TODO: make it work with levels = 3
    //         let position = ACoord { x: 1, y: 1 };
    //         let target = ACoord { x: 9, y: 3 };

    //         let position = ACoord { x: 1, y: 1 };
    //         let target = ACoord { x: 10, y: 4 };
    //         // works with levels = 2 for d*lite
    //         // let position = ACoord { x: 3, y: 2 };
    //         // let target = ACoord { x: 18, y: 3 };
    //         let environment = QuadTree::init(levels);
    //         println!("environment\n{:?}", environment);
    //         // let environment = Grid::new();
    //         let lidar = Lidar::new(12, g_oracle.clone());
    //         // let mut sabby = Sabrina::new(position, environment, lidar, BestFirstPlanner);
    //         // let mut sabby = Sabrina::new(position, environment, lidar, AStarPlanner);
    //         // let mut sabby = Sabrina::new(position, environment, lidar, DStarPlanner::new());
    //         let mut sabby = Sabrina::new(position, environment, lidar, DStarPlanner::new());
    //         println!("absolute_environment\n{q_oracle}");
    //         // println!("-------------------------------");
    //         // println!("    Starting Navigation        ");
    //         // println!("-------------------------------");
    //         println!("Final Status {:?}", sabby.navigate(target));
    //         println!("Final map\n{}", sabby.environment);
    //     }
    //     _ => {
    //         println!("Err");
    //     }
    // }
}
// fn main() {
//     println!("------------------------------------");
//     println!("      Example navigation            ");
//     println!("------------------------------------");
//     let levels = 2;
//     let mut environment = QuadTree::init(levels);
//     println!("environment\n{:?}", environment);
//     let source = ACoord { x: 0, y: 0 };
//     let target = ACoord { x: 5, y: 0 };
//     let source = ACoord { x: 1, y: 1 };
//     let target = ACoord { x: 1, y: 5 };
//     environment.initialize(source, target);
//     println!("--------------------");
//     println!("updated environment\n{:?}", environment);
//     for n in environment.neighbors(HCoord { l:1, x: 0, y: 0 }) {
//         println!("n {n:?}");
//     }
// }
