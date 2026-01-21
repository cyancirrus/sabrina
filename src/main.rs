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
    // space is double to avoid halfints on the quadtree for centroids
    let dh = 1 << ((m_coord.0 >> PARTITION) + 1);
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
    let level = morton.0 >> PARTITION;
    [
        (
            (morton.0 - (1 << PARTITION)) | 1 << (level + 1),
            (morton.1 - (1 << PARTITION)),
        ),
        (
            (morton.0 - (1 << PARTITION)) | 1 << (level + 1),
            (morton.1 - (1 << PARTITION)) | 1 << level,
        ),
    ]
}

// pub fn east_morton(morton: &Coord) -> [Coord; 2] {
//     // child filtered east neighbors; dx := 1
//     let level = morton.0 >> PARTITION;
//     let span = 1 + (1<<level);
//     let mask = 1<< (level-1);
//     let point = point(morton);
//     println!("level {level:?}");
//     [
//         (point.0 + span, point.1 - 1 << (level - 1)),
//         (point.0 + span, point.1 + 1 << (level - 1)),
//         // (
//         //     (morton.0 - (1 << PARTITION)) | 1 << level,
//         //     (morton.1 - (1 << PARTITION)),
//         // ),
//         // (
//         //     (morton.0 - (1 << PARTITION)) | 1 << level,
//         //     (morton.1 - (1 << PARTITION)) | 1 << level,
//         // ),
//     ]
//     // [
//     //     (
//     //         (morton.0 - (1 << PARTITION)) | 1 << level,
//     //         (morton.1 - (1 << PARTITION)),
//     //     ),
//     //     (
//     //         (morton.0 - (1 << PARTITION)) | 1 << level,
//     //         (morton.1 - (1 << PARTITION)) | 1 << level,
//     //     ),
//     // ]
// }
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

fn edge_neighbors(quad: &QuadTree, m_coord: &Coord) -> Vec<Coord> {
    // neighbor and filter need to be opposites ie (neigh east -> filter west);
    let cardinals = cardinals(m_coord);
    println!("Cardinals {cardinals:?}");
    let filters = [west_morton, south_morton, east_morton, north_morton];
    let level = m_coord.0 >> PARTITION;
    let mut neighbors = Vec::new();
    let mut stack = Vec::new();
    let mut found;
    for (cardinal, filter) in cardinals.iter().zip(filters.iter()) {
        println!("cardinal: {cardinal:?}");
        found = false;
        for lvl in level..LEVELS {
            println!("level {lvl:?}");
            let p_coord = encode_morton(&cardinal, lvl);
            println!("next after level");
            if quad.information.contains_key(&p_coord) {
                neighbors.push(p_coord);
                found = false;
                break;
            } else if encode_morton(m_coord, lvl) == p_coord {
                break;
            }
        }
        if found {
            println!("found");
            break;
        }
        stack.push(*cardinal);
        while let Some(p_coord) = stack.pop() {
            println!("drilling down");
            if quad.information.contains_key(&p_coord) {
                println!("here");
                neighbors.push(p_coord);
            } else {
                println!("there");
                stack.extend(filter(&p_coord));
            }
            println!("done?");
        }
        println!("Neighbors {neighbors:?}");
    }
    neighbors
}


fn astar(quad:&QuadTree, source:&Coord, target:&Coord) -> Vec<Coord> {
    let mut pqueue: BinaryHeap<MinNode> = BinaryHeap::new();
    let mut cache: HashSet<Coord> = HashSet::new();
    let mut plan:HashMap<Coord, Coord> = HashMap::new();
    cache.insert(*source);
    
    for n in edge_neighbors(quad, source) {
        println!("hello");
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

    let cardinals = cardinals(&(3,3));
    assert_eq!((5, 3), cardinals[0]);
    assert_eq!((3, 5), cardinals[1]);
    assert_eq!((1, 3), cardinals[2]);
    assert_eq!((3, 1), cardinals[3]);
    let origin = (1,1);
    let m_origin = encode_morton(&origin, 1);;
    assert_eq!([(5,1), (5,3)], east_morton(&m_origin));

    // let source = (3,3);
    // let target = (3,5);
    // let source = (9,3);
    // let target = (11,3);

    // let path = "./data/sample/test_quad0.map";
    // match (read_grid(path), read_quad(path, LEVELS)) {
    //     (Ok(oracle_grid), Ok(oracle_quad)) => {
    //         println!("Oracle Quad\n{oracle_quad}");
    //         println!("-------------------------------");
    //         oracle_quad.display_with_levels();
    //         println!("-------------------------------");
    //         // routing with perfect information
    //         let plan = astar(&oracle_quad, &source, &target);
    //         println!("Plan Starting");
    //         for l in plan.iter().rev() {
    //             println!("{l:?}");
    //         }
    //     }
    //     _ => {
    //         println!("Unexpected Error");
    //     }
    // }
}
