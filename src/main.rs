#![allow(unused)]
use sabrina::algo::a_star::{astar, centroid_estimate, edge_neighbors, point};
use sabrina::algo::d_star::{LazyPQueue, Star, dstar_lite};
use sabrina::environment::grid::Grid;
use sabrina::environment::info::reconstruct;
use sabrina::environment::hier::{
    child_hier, decode_hier, encode_hier, grid_hier, print_hier,
};
use sabrina::environment::quad::{QuadTree, QuadNode};
use sabrina::global::consts::{LEVELS, PARTITION};
use sabrina::global::types::{Belief, Bounds, Coord, KeyNode, MinNode};
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::read_grid;
use sabrina::parser::quad::read_quad;
use sabrina::sensor::lidar::Lidar;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::mem;
use std::time::Instant;

fn main() {
        let levels = 2;
        let mut information = HashMap::new();
        // level 0 contains no shift and level is inclusive
        let stride = 1 << (levels - 1);
        information.insert(
            encode_hier(&(0, 0), levels - 1),
            QuadNode {
                belief: Belief::Unknown,
                homogenous: true,
            },
        );
        let padding = Bounds {
            min_x: 0,
            min_y: 0,
            max_x: stride - 1,
            max_y: stride - 1,
        };
        let seen = Bounds {
            min_x: usize::MAX,
            min_y: usize::MAX,
            max_x: 0,
            max_y: 0,
        };
        let mut map = QuadTree {
            levels: levels,
            seen,
            padding,
            information,
        };
    map.insert_cell(&(1,1), Belief::Occupied);
    debug_assert_eq!(map.information.len(), 4);
    map.insert_cell(&(1,0), Belief::Occupied);
    debug_assert_eq!(map.information.len(), 4);
    map.insert_cell(&(0,0), Belief::Occupied);
    debug_assert_eq!(map.information.len(), 4);
    map.insert_cell(&(0,1), Belief::Occupied);
    debug_assert_eq!(map.get_cell(&(1,1)), Some((1, Belief::Occupied)));
    debug_assert_eq!(map.information.len(), 1);
    map.insert_cell(&(1,1), Belief::Free);
    debug_assert_eq!(map.information.len(), 4);
    map.insert_cell(&(1,0), Belief::Free);
    map.insert_cell(&(0,0), Belief::Free);
    map.insert_cell(&(0,1), Belief::Free);
    debug_assert_eq!(map.information.len(), 1);
    debug_assert_eq!(map.get_cell(&(1,1)), Some((1, Belief::Free)));

}
