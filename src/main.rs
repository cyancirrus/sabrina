#![allow(unused)]
use sabrina::environment::morton::{child_morton, encode_morton, grid_morton, print_morton};
use sabrina::environment::quad::QuadTree;
use sabrina::global::consts::{PARTITION, LEVELS};
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

fn point(m_coord:&Coord) -> Coord {
    // retrieves the centroid scaled by two two prevent half-integers
    let mask = (1 << PARTITION) - 1;
    (
        ((m_coord.0 & mask) << 1) + (1 << (m_coord.0 >> PARTITION)),
        ((m_coord.1 & mask) << 1) + (1 << (m_coord.1 >> PARTITION)),
    )
}
fn centroid_estimate(sm_coord:&Coord,tm_coord:&Coord) -> usize {
    // distance between source-centroid and target-centroid
    let (s_centr, t_centr) = (point(sm_coord), point(tm_coord));
    s_centr.0.abs_diff(t_centr.0) + s_centr.1.abs_diff(t_centr.1)
}

fn cardinals(m_coord:&Coord) -> [Coord;4] {
    let dh = 1 << (m_coord.0 >> PARTITION);
    // clockwise e,n,w,s
    [
        (m_coord.0 + dh, m_coord.1),
        (m_coord.0, m_coord.1 + dh),
        (m_coord.0 - dh, m_coord.1),
        (m_coord.0, m_coord.1 - dh),
    ]
}
fn cardinal_bounding(m_coord:&Coord) -> [Coord;4] {
    // clockwise e,n,w,s
    // find the largest bounding box which does not contain coord
    let level = m_coord.0 >> PARTITION;
    let mask = (1 << PARTITION) - 1;
    let point = (mask & m_coord.0, mask & m_coord.1);
    let mut cards = cardinals(m_coord);
    for dh in 0..4 {
        for lvl in level+1..LEVELS {
            let p_coord = encode_morton(&cards[dh], lvl);
            if encode_morton(m_coord, lvl) == p_coord { break; }
            cards[dh] = p_coord;
        }
    }
    cards
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
        (
            (morton.0 - (1 << PARTITION)),
            (morton.1 - (1 << PARTITION))
        ),
        (
            (morton.0 - (1 << PARTITION)) | 1 << level,
            (morton.1 - (1 << PARTITION)),
        ),
    ]
}

fn edge_neighbors(quad:&QuadTree, m_coord:&Coord) -> Vec<Coord> {
    // Neighbor and filter need to be opposites ie (neigh east -> filter west);
    // east ~ dx = 1;
    // north ~ dy = 1;
    // south ~ dy = 0;
    // west ~ dx = 0;
    let cardinals = cardinals(m_coord);
    let mut neighbors = Vec::new();
    let mut stack = vec![cardinals[0]];
    // East
    while let Some(p_coord) = stack.pop() {
        if let Some(_) = quad.information.get(&p_coord) {
            neighbors.push(p_coord);
        }
        neighbors.extend(west_morton(&p_coord));
    }
    //North
    stack.push(cardinals[1]);
    while let Some(p_coord) = stack.pop() {
        if let Some(_) = quad.information.get(&p_coord) {
            neighbors.push(p_coord);
        }
        neighbors.extend(south_morton(&p_coord));
    }
    //West
    stack.push(cardinals[2]);
    while let Some(p_coord) = stack.pop() {
        if let Some(_) = quad.information.get(&p_coord) {
            neighbors.push(p_coord);
        }
        neighbors.extend(east_morton(&p_coord));
    }
    //South
    stack.push(cardinals[3]);
    while let Some(p_coord) = stack.pop() {
        if let Some(_) = quad.information.get(&p_coord) {
            neighbors.push(p_coord);
        }
        neighbors.extend(south_morton(&p_coord));
    }
    neighbors
}

fn main() {
    let x = encode_morton(&(0,0), 1);
    assert_eq!((2, 2), point(&x));
    let x = encode_morton(&(0,0), 0);
    assert_eq!((1, 1), point(&x));
    let x = encode_morton(&(1,1), 0);
    assert_eq!((3, 3), point(&x));
    let x = encode_morton(&(3,3), 2);
    assert_eq!((4, 4), point(&x));
    let x = encode_morton(&(3,0), 1);
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
