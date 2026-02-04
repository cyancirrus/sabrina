// use sabrina::algo::a_star::AStarPlanner;
// use sabrina::algo::best_first::BestFirstPlanner;
use sabrina::algo::d_star::DStarPlanner;
use sabrina::environment::grid::Grid;
use sabrina::global::types::ACoord;
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::read_grid;
use sabrina::sensor::lidar::Lidar;

fn main() {
    println!("------------------------------------");
    println!("      Example navigation            ");
    println!("------------------------------------");
    let path = "./data/sample/test_nav0.map";
    match read_grid(path) {
        Ok(oracle) => {
            let position = ACoord { x: 1, y: 1 };
            let target = ACoord { x: 18, y: 3 };
            let environment = Grid::new();
            let lidar = Lidar::new(12, oracle.clone());
            // let mut sabby = Sabrina::new(position, environment, lidar, BestFirstPlanner);
            // let mut sabby = Sabrina::new(position, environment, lidar, AStarPlanner);
            let mut sabby = Sabrina::new(position, environment, lidar, DStarPlanner::new());
            println!("absolute_environment\n{oracle}");
            println!("-------------------------------");
            println!("    Starting Navigation        ");
            println!("-------------------------------");
            println!("Final Status {:?}", sabby.navigate(target));
            println!("Final map\n{}", sabby.environment);
        }
        Err(e) => {
            println!("Err\n{e:?}");
        }
    }
}
