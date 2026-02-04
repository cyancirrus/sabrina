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

fn main() {
    println!("------------------------------------");
    println!("      Example navigation            ");
    println!("------------------------------------");
    let path = "./data/sample/test_nav0.map";
    // let path = "./data/sample/test_nav1.map";
    let levels = 2;
    match (read_quad(path, levels), read_grid(path)) {
        (Ok(q_oracle), Ok(g_oracle)) => {
            // println!("------------------------------------");
            // println!("absolute_environment\n{q_oracle}");
            // println!("------------------------------------");
            // println!("absolute_environment\n{q_oracle:?}");
            // println!("------------------------------------");
            // let p = HCoord { l: 0, x: 1, y: 1 };
            // for n in q_oracle.neighbors(p) {
            //     println!("n {n:?}");
            // }
            // return;
            println!("000000000000000000000000000000000");
            let mut environment = QuadTree::init(levels);
            // environment.insert_ray(ACoord{x:0, y:0}, ACoord{x:0, y:1});
            // environment.update_belief(&ACoord{x:0, y:0}, Belief::Unknown);
            // println!("Bounds {:?}", environment.bounds);
            // println!("Environment\n{environment:}");
            // println!("Environment\n{environment:?}");






            // let position = ACoord { x: 1, y: 1 };
            // let target = ACoord { x: 1, y: 3 };

            let position = ACoord { x: 1, y: 1 };
            // let target = ACoord { x: 18, y: 3 };
            let target = ACoord { x: 1, y: 5 };
            let environment = QuadTree::new();
            let lidar = Lidar::new(12, g_oracle.clone());
            // let mut sabby = Sabrina::new(position, environment, lidar, BestFirstPlanner);
            let mut sabby = Sabrina::new(position, q_oracle.clone(), lidar, AStarPlanner);
            // let mut sabby = Sabrina::new(position, environment, lidar, DStarPlanner::new());
            println!("absolute_environment\n{q_oracle}");
            // println!("-------------------------------");
            // println!("    Starting Navigation        ");
            // println!("-------------------------------");
            println!("Final Status {:?}", sabby.navigate(target));
            // println!("Final map\n{}", sabby.environment);
        }
        _ => {
            println!("Err");
        }
    }
}
