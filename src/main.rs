#![allow(unused)]
use sabrina::environment::morton::{child_morton, encode_morton, grid_morton, print_morton};
use sabrina::environment::quad::QuadTree;
use sabrina::global::consts::{LEVELS, PARTITION};
use sabrina::global::types::{Belief, Coord};
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::readmap;
use sabrina::parser::quad::readquad;
use sabrina::sensor::lidar::Lidar;
use std::collections::HashMap;

// // TODO: Bit-level Quadtree Fixes (Off-by-one/Masks)
// // Unknown State Initialization + Visual Debugger (Essential for visibility)
// // Observation Logic (Unknown \(\rightarrow \) Free/Occupied).LU Pivoting (Numerical insurance)
// // Multi-ray LiDAR & Planner Implementation.
// // Hestereses or defered clean up
// // Consdier implementing a jump iter

// okay this makes sense, so now i just need a couple of functions
// - second if it is at lower level of granularity use the those connected by the edge and ensure that we recurse until we find the nodes which actually exist
// - lastly implement a*

fn point(m_coord: &Coord) -> Coord {
    // retrieves the centroid scaled by two two prevent half-integers
    let mask = (1 << PARTITION) - 1;
    (
        ((m_coord.0 & mask) << 1) + (1 << (m_coord.0 >> PARTITION)),
        ((m_coord.1 & mask) << 1) + (1 << (m_coord.1 >> PARTITION)),
    )
}
fn centroid_estimate(sm_coord: &Coord, tm_coord: &Coord) -> usize {
    // distance between source-centroid and target-centroid
    let (s_centr, t_centr) = (point(sm_coord), point(tm_coord));
    s_centr.0.abs_diff(t_centr.0) + s_centr.1.abs_diff(t_centr.1)
}

fn cardinals(m_coord: &Coord) -> [Coord; 4] {
    let dh = 1 << (m_coord.0 >> PARTITION);
    // clockwise e,n,w,s
    [
        (m_coord.0 + dh, m_coord.1),
        (m_coord.0, m_coord.1 + dh),
        (m_coord.0 - dh, m_coord.1),
        (m_coord.0, m_coord.1 - dh),
    ]
}

pub fn east_morton(morton: &Coord) -> [Coord; 2] {
    // child filtered east neighbors; dx := 1
    let level = (morton.0 >> PARTITION) - 1;
    [
        (
            (morton.0 - (1 << PARTITION)) | 1 << level,
            (morton.1 - (1 << PARTITION)),
        ),
        (
            (morton.0 - (1 << PARTITION)) | 1 << level,
            (morton.1 - (1 << PARTITION)) | 1 << level,
        ),
    ]
}
pub fn north_morton(morton: &Coord) -> [Coord; 2] {
    // child filtered west neighbors; dy := 1
    let level = (morton.0 >> PARTITION) - 1;
    [
        (
            (morton.0 - (1 << PARTITION)),
            (morton.1 - (1 << PARTITION)) | 1 << level,
        ),
        (
            (morton.0 - (1 << PARTITION)) | 1 << level,
            (morton.1 - (1 << PARTITION)) | 1 << level,
        ),
    ]
}
pub fn west_morton(morton: &Coord) -> [Coord; 2] {
    // child filtered west neighbors; dx := 0
    let level = (morton.0 >> PARTITION) - 1;
    [
        ((morton.0 - (1 << PARTITION)), (morton.1 - (1 << PARTITION))),
        (
            (morton.0 - (1 << PARTITION)),
            (morton.1 - (1 << PARTITION)) | 1 << level,
        ),
    ]
}
pub fn south_morton(morton: &Coord) -> [Coord; 2] {
    // child filtered south neighbors; dy := 0
    let level = (morton.0 >> PARTITION) - 1;
    [
        ((morton.0 - (1 << PARTITION)), (morton.1 - (1 << PARTITION))),
        (
            (morton.0 - (1 << PARTITION)) | 1 << level,
            (morton.1 - (1 << PARTITION)),
        ),
    ]
}

fn edge_up(m_coord: &Coord) -> [Coord; 4] {
    // clockwise e,n,w,s
    // find the largest bounding box which does not contain coord
    let level = m_coord.0 >> PARTITION;
    let mask = (1 << PARTITION) - 1;
    let mut cards = cardinals(m_coord);
    for dh in 0..4 {
        for lvl in level + 1..LEVELS {
            let p_coord = encode_morton(&cards[dh], lvl);
            if encode_morton(m_coord, lvl) == p_coord {
                break;
            }
            cards[dh] = p_coord;
        }
    }
    cards
}

fn edge_neighbors(quad: &QuadTree, m_coord: &Coord) -> Vec<Coord> {
    // neighbor and filter need to be opposites ie (neigh east -> filter west);
    let cardinals = cardinals(m_coord);
    let filters = [west_morton, south_morton, east_morton, north_morton];
    let level = m_coord.0 >> PARTITION;
    let mut neighbors = Vec::new();
    let mut stack = Vec::new();
    let mut found;
    for (cardinal, filter) in cardinals.iter().zip(filters.iter()) {
        found = false;
        for lvl in level..LEVELS {
            let p_coord = encode_morton(&cardinal, lvl);
            if quad.information.contains_key(&p_coord) {
                neighbors.push(p_coord);
                found = false;
                break;
            } else if encode_morton(m_coord, lvl) == p_coord {
                break;
            }
        }
        if found {
            break;
        }
        stack.push(*cardinal);
        while let Some(p_coord) = stack.pop() {
            if quad.information.contains_key(&p_coord) {
                neighbors.push(p_coord);
            } else {
                stack.extend(filter(&p_coord));
            }
        }
    }
    neighbors
}

// TODO: split this into going up search and going down
fn edge_down(quad: &QuadTree, m_coord: &Coord) -> Vec<Coord> {
    // neighbor and filter need to be opposites ie (neigh east -> filter west);
    let cardinals = cardinals(m_coord);
    let filters = [west_morton, south_morton, east_morton, north_morton];
    let mut neighbors = Vec::new();
    let mut stack = Vec::new();
    for (cardinal, filter) in cardinals.iter().zip(filters.iter()) {
        stack.push(*cardinal);
        while let Some(p_coord) = stack.pop() {
            if quad.information.contains_key(&p_coord) {
                neighbors.push(p_coord);
            } else {
                stack.extend(filter(&p_coord));
            }
        }
    }
    neighbors
}

fn main() {
    let x = encode_morton(&(0, 0), 1);
    assert_eq!((2, 2), point(&x));
    let x = encode_morton(&(0, 0), 0);
    assert_eq!((1, 1), point(&x));
    let x = encode_morton(&(1, 1), 0);
    assert_eq!((3, 3), point(&x));
    let x = encode_morton(&(3, 3), 2);
    assert_eq!((4, 4), point(&x));
    let x = encode_morton(&(3, 0), 1);
    assert_eq!((6, 2), point(&x));

    // // let mut quad = QuadTree::new(8, 4, 4);
    // let mut quad = QuadTree::new(4, 4, 3);
    // quad.display_with_levels();
    // println!("----------------------------------");
    // quad.insert_cell(&(1, 1), Belief::Occupied);
    // quad.display_with_levels();
    // println!("----------------------------------");
    // println!("{quad}");
    // println!("----------------------------------");
    // quad.insert_cell(&(0, 0), Belief::Occupied);
    // quad.insert_cell(&(0, 1), Belief::Occupied);
    // quad.insert_cell(&(1, 0), Belief::Occupied);
    // println!("----------------------------------");
    // quad.display_with_levels();

    // let path = "./data/sample/test_quad0.map";
    // match (readmap(path), readquad(path, consts::LEVELS)) {
    //     (Ok(oracle_grid), Ok(oracle_quad)) => {
    //         println!("Oracle Quad\n{oracle_quad}");
    //         println!("-------------------------------");
    //         println!("Oracle Quad nodes\n{:?}", oracle_quad.information.len());
    //         println!("-------------------------------");
    //         oracle_quad.display_with_levels();
    //     }
    //     _ => {
    //         println!("Unexpected Error");
    //     }
    // }
}
