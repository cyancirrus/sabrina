use crate::environment::info::reconstruct;
use crate::environment::quad::QuadTree;
use crate::global::types::{AStarPlan, Belief, Coord, HeurNode};
use crate::hierarchy::encoding::centroid_estimate;
use crate::hierarchy::proximity::edge_neighbors;
use std::collections::{BinaryHeap, HashMap};
// // Observation Logic (Unknown \(\rightarrow \) Free/Occupied).LU Pivoting (Numerical insurance)
// // Multi-ray LiDAR & Planner Implementation.
// // Hestereses or defered clean up
// // Consdier implementing a jump iter

// TODO: Think through whether the boundary case exists where we aren't surrounded by wall

pub fn astar_precursor(quad: &QuadTree, source: Coord, target: Coord) -> HashMap<Coord, Coord> {
    let mut pqueue: BinaryHeap<HeurNode<Coord>> = BinaryHeap::new();
    let mut adjacency: HashMap<Coord, Coord> = HashMap::new();
    let mut best_cost: HashMap<Coord, usize> = HashMap::new();
    best_cost.insert(source, 0);
    adjacency.insert(source, source);

    pqueue.push(HeurNode {
        coord: source,
        cost: centroid_estimate(source, target),
        incurred: 0,
    });
    while let Some(node) = pqueue.pop() {
        if node.coord == target {
            break;
        }
        for new_coord in edge_neighbors(quad, node.coord) {
            if quad.information[&new_coord].belief == Belief::Occupied {
                continue;
            }
            let known_cost = node.incurred + centroid_estimate(new_coord, node.coord);
            let heuristic = centroid_estimate(new_coord, target);
            if known_cost >= *best_cost.get(&new_coord).unwrap_or(&usize::MAX) {
                continue;
            }
            best_cost.insert(new_coord, known_cost);
            pqueue.push(HeurNode {
                coord: new_coord,
                cost: known_cost + heuristic,
                incurred: known_cost,
            });
            adjacency.insert(new_coord, node.coord);
        }
    }
    adjacency
}

pub fn astar(quad: &QuadTree, source: Coord, target: Coord) -> AStarPlan {
    let precursor = astar_precursor(quad, source, target);
    AStarPlan {
        plan: reconstruct(&precursor, source, target),
    }
}

// pub fn main() {
//     let origin = (4, 4);
//     let x = encode_hier(&origin, 1);
//     assert_eq!(x, encode_hier(&x, 1));
//     assert_eq!(encode_hier(&origin, 1), encode_hier(&x, 1));
//     assert_eq!(encode_hier(&origin, 2), encode_hier(&x, 2));
//     // point is interface only for distance, does not maintain scale
//     let x = encode_hier(&(1, 1), 0);
//     assert_eq!((3, 3), point(x));
//     let x = encode_hier(&(2, 2), 0);
//     assert_eq!((5, 5), point(x));
//     let x = encode_hier(&(1, 3), 0);
//     assert_eq!((3, 7), point(x));
//     // -------------
//     let x = encode_hier(&(0, 0), 1);
//     assert_eq!((2, 2), point(x));
//     let x = encode_hier(&(1, 0), 1);
//     assert_eq!((2, 2), point(x));
//     let x = encode_hier(&(2, 0), 1);
//     assert_eq!((6, 2), point(x));
//     let x = encode_hier(&(3, 3), 2);
//     assert_eq!((4, 4), point(x));

//     let cardinals = find_cardinals((2, 2));
//     assert_eq!((3, 2), cardinals[0]);
//     assert_eq!((2, 3), cardinals[1]);
//     assert_eq!((1, 2), cardinals[2]);
//     assert_eq!((2, 1), cardinals[3]);

//     let x = encode_hier(&(2, 2), 1);
//     let cardinals = find_cardinals(x);
//     assert_eq!(encode_hier(&(5, 2), 1), cardinals[0]);
//     assert_eq!(encode_hier(&(2, 5), 1), cardinals[1]);
//     assert_eq!(encode_hier(&(0, 2), 1), cardinals[2]);
//     assert_eq!(encode_hier(&(2, 0), 1), cardinals[3]);

//     let origin = (0, 0);
//     let m_origin = encode_hier(&origin, 1);
//     assert_eq!([(1, 0), (1, 1)], east_hier(m_origin));
//     assert_eq!([(1, 1), (0, 1)], north_hier(m_origin));
//     assert_eq!([(0, 1), (0, 0)], west_hier(m_origin));
//     assert_eq!([(0, 0), (1, 0)], south_hier(m_origin));

//     let x = encode_hier(&(4, 4), 2);
//     // // south edge
//     assert_eq!(
//         [encode_hier(&(4, 4), 1), encode_hier(&(6, 4), 1)],
//         south_hier(x)
//     );

//     let origin = (4, 2);
//     let m_origin = encode_hier(&origin, 1);
//     assert_eq!([(5, 2), (5, 3)], east_hier(m_origin));
//     assert_eq!([(5, 3), (4, 3)], north_hier(m_origin));
//     assert_eq!([(4, 3), (4, 2)], west_hier(m_origin));
//     assert_eq!([(4, 2), (5, 2)], south_hier(m_origin));

//     let origin = (5, 2);

//     let source = (1, 1);
//     let target = (18, 3);
//     let source = (1, 1);
//     let target = (5, 2);

//     let path = "./data/sample/test_nav0.map";
//     // let path = "./data/sample/test_quad0.map";
//     match (read_grid(path), read_quad(path, LEVELS)) {
//         (Ok(oracle_grid), Ok(oracle_quad)) => {
//             println!("Oracle Quad\n{oracle_quad}");
//             println!("-------------------------------");
//             oracle_quad.display_with_levels();
//             println!("-------------------------------");
//             // routing with perfect information
//             let start = Instant::now();
//             let plan = astar(&oracle_quad, source, target);
//             println!("Duration {:?}", start.elapsed());
//             println!("plan {plan:?}");
//             // println!("Plan Starting");
//             // for l in plan.iter().rev() {
//             //     println!("{l:?}");
//             // }
//             // // println!("Plan Ended");
//         }
//         _ => {
//             println!("Unexpected Error");
//         }
//     }
// }

// // Oracle Quad
// // [#][#][#][ ][#][#][#][#]
// // [#][#][#][ ][#][#][#][#]
// // [#][ ][ ][ ][#][#][#][#]
// // [#][#][#][ ][#][#][#][#]
// // [#][ ][#][ ][ ][ ][#][#]
// // [#][ ][ ][ ][ ][ ][#][#]
// // [#][ ][#][#][ ][ ][ ][ ]
// // [#][#][#][#][#][#][#][#]

// // -------------------------------
// // [1][1][0][0][2][2][2][2]
// // [1][1][0][0][2][2][2][2]
// // [0][0][0][0][2][2][2][2]
// // [0][0][0][0][2][2][2][2]
// // [0][0][0][0][1][1][1][1]
// // [0][0][0][0][1][1][1][1]
// // [0][0][1][1][0][0][0][0]
// // [0][0][1][1][0][0][0][0]
// // -------------------------------
