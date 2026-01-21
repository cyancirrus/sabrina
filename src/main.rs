#![allow(unused)]
use sabrina::global::consts::{LEVELS, PARTITION};
use sabrina::global::types::{Belief, Coord, MinNode};
use sabrina::environment::morton::{child_morton, encode_morton, grid_morton, print_morton};
use sabrina::environment::quad::QuadTree;
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::read_grid;
use sabrina::parser::quad::read_quad;
use sabrina::sensor::lidar::Lidar;
use std::collections::{HashMap, HashSet, BinaryHeap};
use sabrina::environment::info::reconstruct;

// // NOTE: Bit-level Quadtree Fixes (Off-by-one/Masks)
// // Unknown State Initialization + Visual Debugger (Essential for visibility)
// // Observation Logic (Unknown \(\rightarrow \) Free/Occupied).LU Pivoting (Numerical insurance)
// // Multi-ray LiDAR & Planner Implementation.
// // Hestereses or defered clean up
// // Consdier implementing a jump iter

// okay this makes sense, so now i just need a couple of functions
// - second if it is at lower level of granularity use the those connected by the edge and ensure that we recurse until we find the nodes which actually exist
// - lastly implement a*

// TODO: refactor this the double integer scaling just would need a bottom up refactor of entire morton
// interface

fn point(m_coord: &Coord) -> Coord {
    // An interface for retrieving purely for retrieving distance
    // retrieves the centroid scaled by two in order to prevent half-integers
    let level = m_coord.0 >> PARTITION;
        let mask = (1 << PARTITION) - 1;
        (
            ((m_coord.0 & mask) << 1) + (1 << level ),
            ((m_coord.1 & mask) << 1) + (1 << level ),
        )
}
fn centroid_estimate(sm_coord: &Coord, tm_coord: &Coord) -> usize {
    // distance between source-centroid and target-centroid
    let (s_centr, t_centr) = (point(sm_coord), point(tm_coord));
    s_centr.0.abs_diff(t_centr.0) + s_centr.1.abs_diff(t_centr.1)
}

fn find_cardinals(m_coord: &Coord) -> [Coord; 4] {
    // space is double to avoid halfints on the quadtree for centroids
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
            (morton.1 - (1 << PARTITION)) ,
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
            (morton.0 - (1 << PARTITION)) | 1 << level,
            (morton.1 - (1 << PARTITION)) | 1 << level,
        ),
        (
            (morton.0 - (1 << PARTITION)),
            (morton.1 - (1 << PARTITION)) | 1 << level,
        ),
    ]
}
pub fn west_morton(morton: &Coord) -> [Coord; 2] {
    // child filtered west neighbors; dx := 0
    let level = (morton.0 >> PARTITION) - 1;
    [
        (
            (morton.0 - (1 << PARTITION)),
            (morton.1 - (1 << PARTITION)) | 1 << level,
        ),
        (
            (morton.0 - (1 << PARTITION)),
            (morton.1 - (1 << PARTITION)),
        ),
    ]
}
pub fn south_morton(morton: &Coord) -> [Coord; 2] {
    // child filtered south neighbors; dy := 0
    let level = (morton.0 >> PARTITION) - 1;
    [
        (
            (morton.0 - (1 << PARTITION)),
            (morton.1 - (1 << PARTITION)),
        ),
        (
            (morton.0 - (1 << PARTITION)) | 1 << level,
            (morton.1 - (1 << PARTITION)),
        ),
    ]
}

fn edge_neighbors(quad: &QuadTree, m_coord: &Coord) -> Vec<Coord> {
    // neighbor and filter need to be opposites ie (neigh east -> filter west);
    let cardinals = find_cardinals(m_coord);
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
                found = true;
                break;
            } else if encode_morton(m_coord, lvl) == p_coord {
                break;
            }
        }
        if found {
            continue;
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


fn astar(quad:&QuadTree, source:&Coord, target:&Coord) -> Vec<Coord> {
    let mut pqueue: BinaryHeap<MinNode> = BinaryHeap::new();
    let mut cache: HashSet<Coord> = HashSet::new();
    let mut plan:HashMap<Coord, Coord> = HashMap::new();
    cache.insert(*source);
    
    for n in edge_neighbors(quad, source) {
        pqueue.push(MinNode {
            cost: centroid_estimate(source, target),
            coord: n,
        });
        cache.insert(n);
        plan.insert(n, *source);
    }
    println!("initialization good");

    while let Some(n) = pqueue.pop() {
        if n.coord == *target {break;}
        for c in edge_neighbors(quad, &n.coord) {
            if quad.information[&c].belief == Belief::Occupied { continue; }
            pqueue.push(MinNode {
                cost: centroid_estimate(&c, target),
                coord: c,
            });
            plan.insert(c, n.coord);
            cache.insert(c);
        }
    }
    reconstruct(&plan, source, target)
}

fn main() {
    // point is interface only for distance, does not maintain scale
    let x = encode_morton(&(1, 1), 0);
    assert_eq!((3, 3), point(&x));
    let x = encode_morton(&(2, 2), 0);
    assert_eq!((5, 5), point(&x));
    let x = encode_morton(&(1, 3), 0);
    assert_eq!((3, 7), point(&x));
    //-------------
    let x = encode_morton(&(0, 0), 1);
    assert_eq!((2, 2), point(&x));
    let x = encode_morton(&(1, 0), 1);
    assert_eq!((2, 2), point(&x));
    let x = encode_morton(&(2, 0), 1);
    assert_eq!((6, 2), point(&x));
    let x = encode_morton(&(3, 3), 2);
    assert_eq!((4, 4), point(&x));

    let cardinals = find_cardinals(&(2,2));
    assert_eq!((3, 2), cardinals[0]);
    assert_eq!((2, 3), cardinals[1]);
    assert_eq!((1, 2), cardinals[2]);
    assert_eq!((2, 1), cardinals[3]);
    
    let x = encode_morton(&(2,2), 1);
    let cardinals = find_cardinals(&x);
    assert_eq!(encode_morton(&(5,2), 1), cardinals[0]);
    assert_eq!(encode_morton(&(2,5), 1), cardinals[1]);
    assert_eq!(encode_morton(&(0,2), 1), cardinals[2]);
    assert_eq!(encode_morton(&(2,0), 1), cardinals[3]);

    let origin = (0,0);
    let m_origin = encode_morton(&origin, 1);;
    assert_eq!([(1,0), (1,1)], east_morton(&m_origin));
    assert_eq!([(1,1), (0, 1)], north_morton(&m_origin));
    assert_eq!([(0,1), (0,0)], west_morton(&m_origin));
    assert_eq!([(0,0), (1,0)], south_morton(&m_origin));




    let source = (1,1);
    let target = (1,2);

    let path = "./data/sample/test_quad0.map";
    match (read_grid(path), read_quad(path, LEVELS)) {
        (Ok(oracle_grid), Ok(oracle_quad)) => {
            println!("Oracle Quad\n{oracle_quad:?}");
            println!("Oracle Quad\n{oracle_quad}");
            println!("-------------------------------");
            oracle_quad.display_with_levels();
            println!("-------------------------------");
            // routing with perfect information
            let plan = astar(&oracle_quad, &source, &target);
            println!("Plan Starting");
            for l in plan.iter().rev() {
                println!("{l:?}");
            }
        }
        _ => {
            println!("Unexpected Error");
        }
    }
}
