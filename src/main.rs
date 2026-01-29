#![allow(unused)]
use sabrina::algo::a_star::AStarPlanner;
use sabrina::algo::best_first::BestFirstPlanner;
use sabrina::algo::d_star::DStarPlanner;
use sabrina::environment::grid::Grid;
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::read_grid;
use sabrina::sensor::lidar::Lidar;

// Target DStarPlan { plan: [(1, 2), (1, 3), (2, 3), (3, 3), (4, 3), (5, 3), (6, 3), (7, 3), (8, 3), (9, 3), (10, 3), (11, 3), (12, 3), (13, 3), (14, 3), (15, 3), (16, 3), (17, 3), (18, 3)] }

fn main() {
    println!("------------------------------------");
    println!("      Example navigation            ");
    println!("------------------------------------");
    let path = "./data/sample/test_nav0.map";
    // NOTE: Checking items
    let path = "./data/sample/test_nav1.map";
    // let path = "./data/sample/test_imposs.map";
    match read_grid(path) {
        Ok(oracle) => {
            let neighs = oracle.inspect_neighs((1, 1));
            for n in neighs {
                println!("n {n:?}");
            }
            let position = (1, 1);
            let target = (2, 2);
            // let target = (18, 3);
            // let target = (3, 2);
            let environment = Grid::new();
            let lidar = Lidar::new(6, oracle.clone());
            // let mut sabby = Sabrina::new(position, environment, lidar, BestFirstPlanner);
            // let mut sabby = Sabrina::new(position, environment, lidar, AStarPlanner);
            let mut sabby = Sabrina::new(position, oracle.clone(), lidar, DStarPlanner::new());
            // let mut sabby = Sabrina::new(position, environment, lidar, DStarPlanner::new());
            println!("absolute_environment\n{oracle}");
            // println!("-------------------------------");
            // println!("    Starting Navigation        ");
            // println!("-------------------------------");
            println!("Final Status {:?}", sabby.navigate(target));
            println!("Final map\n{}", sabby.environment);
        }
        Err(e) => {
            println!("Err\n{e:?}");
        }
    }
}
