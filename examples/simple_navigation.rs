use sabrina::environment::grid::Grid;
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
            let position = (1, 1);
            let target = (18, 3);
            let environment = Grid::new();
            let lidar = Lidar::new(12, oracle.clone());
            let mut sabby = Sabrina::new(position, environment, lidar);
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
