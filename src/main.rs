#![allow(unused)]
use sabrina::algo::a_star::AStarPlanner;
use sabrina::algo::best_first::BestFirstPlanner;
use sabrina::algo::d_star::DStarPlanner;
use sabrina::environment::grid::Grid;
use sabrina::global::types::ACoord;
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::environment::quad::QuadTree;
use sabrina::parser::quad::read_quad;
use sabrina::parser::grid::read_grid;
use sabrina::sensor::lidar::Lidar;
use sabrina::global::types::{SpatialMap};
use sabrina::global::types::Belief;
use sabrina::global::types::HCoord;


fn test_basic() {
    let levels = 2;
    let mut map = QuadTree::init(levels);
    map.update_belief(&ACoord { x: 1, y: 1 }, Belief::Occupied);
    assert_eq!(map.information.len(), 4);
    map.update_belief(&ACoord { x: 1, y: 0 }, Belief::Occupied);
    assert_eq!(map.information.len(), 4);
    map.update_belief(&ACoord { x: 0, y: 0 }, Belief::Occupied);
    assert_eq!(map.information.len(), 4);
    map.update_belief(&ACoord { x: 0, y: 1 }, Belief::Occupied);
    assert_eq!(
        map.get_coord(ACoord { x: 1, y: 1 }),
        Some((1, Belief::Occupied))
    );
}


fn main() {
    let levels = 2;
    let mut map = QuadTree::init(levels);
    // map.update_belief(&ACoord { x: 1, y: 1 }, Belief::Occupied);
    // println!("----------------");
    // println!("{map:?}");
    // println!("{map:}");
    let source = ACoord {x:0, y:0};
    let target = ACoord {x:15, y:0};
    map.initialize(source, target);

    let test = map.neighbors(HCoord {l: 1, x: 2, y: 0 } );
    println!("----------------------");
    for t in test {
        println!("t {t:?}");
    }
    println!("----------------");
    println!("{map:?}");
    println!("{map:}");

}
