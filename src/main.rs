#![allow(unused)]
use sabrina::algo::a_star::AStarPlanner;
use sabrina::algo::best_first::BestFirstPlanner;
use sabrina::algo::d_star::DStarPlanner;
use sabrina::environment::grid::Grid;
use sabrina::environment::quad::QuadTree;
use sabrina::global::types::Belief;
use sabrina::global::types::SpatialMap;
use sabrina::global::types::{ACoord, HCoord};
use sabrina::hierarchy::encoding::{child_hier, encode, grid_hier, point, transform};
use sabrina::hierarchy::proximity::grid_components;
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::read_grid;
use sabrina::parser::quad::read_quad;
use sabrina::sensor::lidar::Lidar;

// TODO: Just need to check that the env.neighbors gets well with mixed hierarhchy
// TODO: This isn't working with env.neighbors HCoord { l: 2, x: 4, y: 0 }: (16, 16), HCoord { l: 0, x: 3, y: 2 }: (22, 22)
// TODO: Consider for the cardinals to have it go to the largest non-containing node - likely can
// overwrite in edge-neighbors loop
// ERROR in the reverse of the transform b/c it will overwrite data, why reverse, edge numbers
// check git diff
// FATAL: fn components(&self, node: &Self::Encoded) -> Vec<Self::Encoded> {
// TODO: Have like, create dirty reads and hesterisis or whatnot, like
// (new nodes, del nodes, merge nodes)

// fn main() {
//     let levels = 2;
//     let env = QuadTree::init(levels);
//     println!("env {env:?}");
//     let node = HCoord { l: 0, x: 1, y: 0 };

//     for s in grid_components(&node, 0) {
//         println!("{s:?}");
//     }
//     println!("hello world");

//     // let levels = 2;
//     // let env = QuadTree::init(levels);
//     // println!("env {env:?}");
//     // let node = HCoord { l: 0, x: 1, y: 0 };

//     // for s in grid_components(&node, 0) {
//     //     println!("{s:?}");
//     // }
//     // println!("hello world");
// }

fn test_edge_neighbors() {
    let path = "./data/sample/test_nav0.map";
    let levels = 1;
    let env = read_quad(path, levels).unwrap();
}

fn main() {
    println!("------------------------------------");
    println!("      Example navigation            ");
    println!("------------------------------------");
    let path = "./data/sample/test_nav0.map";
    // let path = "./data/sample/test_nav1.map";
    let levels = 3;
    match (read_quad(path, levels), read_grid(path)) {
        (Ok(q_oracle), Ok(g_oracle)) => {
            // works with levels = 5 for d*lite
            let position = ACoord { x: 1, y: 1 };
            let target = ACoord { x: 1, y: 5 };

            // // // TODO: make it work with levels = 3
            let position = ACoord { x: 1, y: 1 };
            let target = ACoord { x: 9, y: 3 };

            let position = ACoord { x: 5, y: 2 };
            let target = ACoord { x: 9, y: 3 };

            let position = ACoord { x: 3, y: 1 };
            let target = ACoord { x: 10, y: 4 };

            let position = ACoord { x: 1, y: 1 };
            let target = ACoord { x: 15, y: 7 };

            let position = ACoord { x: 1, y: 1 };
            let target = ACoord { x: 18, y: 3 };
            
            let position = ACoord { x: 5, y: 2 };
            let target = ACoord { x: 18, y: 3 };
            let environment = QuadTree::init(levels);
            println!("environment\n{:?}", environment);
            // let environment = Grid::new();
            let lidar = Lidar::new(12, g_oracle.clone());
            // let mut sabby = Sabrina::new(position, environment, lidar, BestFirstPlanner);
            // let mut sabby = Sabrina::new(position, environment, lidar, AStarPlanner);
            let mut sabby = Sabrina::new(position, environment, lidar, DStarPlanner::new());
            // let mut sabby = Sabrina::new(position, q_oracle.clone(), lidar, DStarPlanner::new());
            println!("absolute_environment\n{q_oracle}");
            // println!("-------------------------------");
            // println!("    Starting Navigation        ");
            // println!("-------------------------------");
            println!("Final Status {:?}", sabby.navigate(target));
            println!("Final map\n{}", sabby.environment);
            println!("Final map\n{:?}", sabby.environment);
        }
        _ => {
            println!("Err");
        }
    }
}
