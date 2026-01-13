#![allow(unused)]
use sabrina::environment::representation::{Bounds, Environment};
use sabrina::parser::map::readmap;
use sabrina::sensor::lidar::Lidar;
use sabrina::intelligence::sabrina::Sabrina;
use std::collections::{HashMap};

fn main() {
    println!("------------------------------------");
    println!("      Example navigation            ");
    println!("------------------------------------");
    let path = "./data/sample/test_nav0.map";
    match readmap(path) {
        Ok(oracle) => {
            let position = (1, 1);
            let target = (18, 3);
            // let target = (4, 2);
            // let target = (1, 2);
            // println!("Test oracle pathclear 0, 1 {:?}", oracle.path_clear(&(1,2)));
            let bounds = Bounds::new(0, 0, 4, 4);
            let environment = Environment::new(HashMap::new(), bounds);
            let lidar = Lidar::new(100, oracle.clone());
            let mut sabby = Sabrina::new(position, environment, lidar);
            println!("absolute_environment\n{oracle}");
            println!("-------------------------------");
            println!("    Starting Navigation        ");
            println!("-------------------------------");

            // sabby.scan();
            println!("Final Status {:?}", sabby.navigate(target));
            println!("Final map\n{}", sabby.environment);
        }
        Err(e) => {
            println!("Err\n{e:?}");
        }
    }
}
