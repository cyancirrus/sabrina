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


// pub fn transform(coord: &HCoord, level: usize) -> HCoord {
//     if level < coord.l {
//         return *coord;
//     }
//     let mask = !((1 << level) - 1);
//     HCoord {
//         l: level,
//         x: coord.x & mask,
//         y: coord.y & mask,
//     }
// }

// pub fn encode(coord: ACoord, level: usize) -> HCoord {
//     let mask = !((1 << level) - 1);
//     HCoord {
//         l: level,
//         x: coord.x & mask,
//         y: coord.y & mask,
//     }
// }

// fn test_encode() {
//     let x = ACoord {x:0, y: 2};
//     let eresult = HCoord {l: 1, x: 0, y: 2};
//     assert_eq!(eresult, encode(x, 1));
    
//     let x = ACoord {x:0, y: 2};
//     let eresult = HCoord {l: 0, x: 0, y: 2};
//     assert_eq!(eresult, encode(x, 0));
// }

// // pub fn get_node(&self, mut node: HCoord) -> Option<(usize, Belief)> {
// //     for lvl in 0..self.levels {
// //         node = transform(&node, lvl);
// //         println!("Get Node {node:?}");
// //         if let Some(n) = self.information.get(&node) {
// //             if n.homogenous {
// //                 return Some((lvl, n.belief));
// //             }
// //         }
// //     }
// //     None
// // }

// fn test_transform() {
//     let x = HCoord { l: 0, x: 2, y: 0 };
//     let eresult = HCoord{ l: 1, x: 2, y: 0 };
//     assert_eq!(eresult, transform(&x, 1));
    
//     let x = HCoord { l: 0, x: 2, y: 0 };
//     let eresult = HCoord{ l: 0, x: 2, y: 0 };
//     assert_eq!(eresult, transform(&x, 0));
    
//     let x = HCoord { l: 1, x: 4, y: 0 };
//     let eresult = HCoord{ l: 2, x: 4, y: 0 };
//     assert_eq!(eresult, transform(&x, 2));
// }

// fn main() {
//     test_transform();
//     test_encode();
// }

// TODO: We need to update the vertex for all of the subgrids
// TODO: think i need to call the update for the grid neighbors as well as edge neighbors
// fn main() {
//     use sabrina::hierarchy::proximity::edge_neighbors;
//     println!("------------------------------------");
//     println!("      Example navigation            ");
//     println!("------------------------------------");
//     let path = "./data/sample/test_nav0.map";
//     // let path = "./data/sample/test_nav1.map";
//     let levels = 2;
//     let oracle = read_quad(path, levels).unwrap();
//     // let x = HCoord{ l:0, x: 3, y: 2 };
//     let x = HCoord{ l:1, x: 4, y: 0 };
//     for n in edge_neighbors(&oracle, x) {
//         println!("n {n:?}");
//     }
//     println!("{oracle:}");
// }

fn main() {
    println!("------------------------------------");
    println!("      Example navigation            ");
    println!("------------------------------------");
    let path = "./data/sample/test_nav0.map";
    // let path = "./data/sample/test_nav1.map";
    let levels = 2;
    match (read_quad(path, levels), read_grid(path)) {
        (Ok(q_oracle), Ok(g_oracle)) => {
            // works with levels = 5 for d*lite
            let position = ACoord { x: 1, y: 1 };
            let target = ACoord { x: 1, y: 5 };

            // // // // TODO: make it work with levels = 3
            // let position = ACoord { x: 1, y: 1 };
            // let target = ACoord { x: 9, y: 3 };

            // let position = ACoord { x: 1, y: 1 };
            // let target = ACoord { x: 10, y: 4 };
            // works with levels = 2 for d*lite
            // let position = ACoord { x: 1, y: 1 };
            

            let position = ACoord { x: 3, y: 2 };
            let target = ACoord { x: 18, y: 3 };
            let environment = QuadTree::init(levels);
            println!("environment\n{:?}", environment);
            // let environment = Grid::new();
            let lidar = Lidar::new(12, g_oracle.clone());
            // let mut sabby = Sabrina::new(position, environment, lidar, BestFirstPlanner);
            // let mut sabby = Sabrina::new(position, environment, lidar, AStarPlanner);
            // let mut sabby = Sabrina::new(position, environment, lidar, DStarPlanner::new());
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
