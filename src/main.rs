#![allow(unused)]
use sabrina::algo::a_star::{astar, centroid_estimate, edge_neighbors, point};
use sabrina::algo::d_star::{LazyPQueue, Star, dstar_lite};
use sabrina::environment::grid::Grid;
use sabrina::environment::info::reconstruct;
use sabrina::environment::morton::{
    child_morton, decode_morton, encode_morton, grid_morton, print_morton,
};
use sabrina::environment::quad::QuadTree;
use sabrina::global::consts::{LEVELS, PARTITION};
use sabrina::global::types::{Belief, Bounds, Coord, KeyNode, MinNode};
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::read_grid;
use sabrina::parser::quad::read_quad;
use sabrina::sensor::lidar::Lidar;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::mem;
use std::time::Instant;

type MinHeap = BinaryHeap<MinNode>;
// TODO: refactoring from usize -> isize caused neighbors to not be propegated properly
fn main() {
    let origin = (5, 2);

    let source = (1, 1);
    let target = (18, 3);
    let source = (1, 1);
    // let target = (5, 2);
    // let target = (5, 1);
    let target = (3, 6);
    // let target = (1, 3);
    let source = (1, 1);
    let target = (18, 3);

    let path = "./data/sample/test_nav0.map";
    // let path = "./data/sample/test_quad0.map";
    match read_quad(path, LEVELS) {
        Ok(oracle_quad) => {
            println!("Oracle Quad\n{oracle_quad}");
            println!("-------------------------------");
            oracle_quad.display_with_levels();
            println!("-------------------------------");
            let mut star = Star::new();
            let mut update = LazyPQueue::new();
            let start = Instant::now();
            // let plan = astar(&oracle_quad, source, target);
            let plan = dstar_lite(&oracle_quad, &mut star, &mut update, source, target);
            println!("Duration D*Lite {:?}", start.elapsed());

            let start = Instant::now();
            let _ = astar(&oracle_quad, source, target);
            println!("Duration A* {:?}", start.elapsed());
            println!("------------------------------");
            println!("------------------------------");
            println!("--------------");
            println!("Plan ");
            for p in plan {
                let (x, y) = decode_morton(p);
                print!("({}, {})), ", x, y);
            }
            print!("\n");
        }
        _ => {
            println!("Unexpected Error");
        }
    }
}
