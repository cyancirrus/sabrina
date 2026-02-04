use crate::environment::new_quad::QuadTree;
use crate::global::types::{Belief, HCoord};
use crate::hierarchy::new_encoding::{transform};

// // Observation Logic (Unknown \(\rightarrow \) Free/Occupied).LU Pivoting (Numerical insurance)
// // Multi-ray LiDAR & Planner Implementation.
// // Hestereses or defered clean up
// // Consdier implementing a jump iter

// TODO: Think through whether the boundary case exists where we aren't surrounded by wall

/// Finds north south east west not the quadgrid
const EDGE_FILTERS:[fn(HCoord) -> [HCoord;2]; 4] = [west_hier, south_hier, east_hier, north_hier];

pub fn find_cardinals(node: HCoord) -> [HCoord; 4] {
    // space is double to avoid halfints on the quadtree for centroids
    let dh = 1 << node.l;
    // clockwise e,n,w,s
    [
        HCoord {
            l: node.l,
            x: node.x + dh,
            y: node.y,
        },
        HCoord {
            l: node.l,
            x: node.x,
            y: node.y + dh,
        },
        HCoord {
            l: node.l,
            x: node.x - dh,
            y: node.y,
        },
        HCoord {
            l: node.l,
            x: node.x - dh,
            y: node.y,
        },
    ]
}
/// child nodes filter east neighbors; dx := 1
pub fn east_hier(node: HCoord) -> [HCoord; 2] {
    let l = node.l;
    let dh = 1 << l;
    [
        HCoord {
            l: l,
            x: node.x | dh,
            y: node.y,
        },
        HCoord {
            l: l,
            x: node.x | dh,
            y: node.y | dh,
        },
    ]
}
/// child filtered west neighbors; dy := 1
pub fn north_hier(node: HCoord) -> [HCoord; 2] {
    let l = node.l;
    let dh = 1 << l;
    [
        HCoord {
            l: l,
            x: node.x | dh,
            y: node.y | dh,
        },
        HCoord {
            l: l,
            x: node.x,
            y: node.y | dh,
        },
    ]
}
/// child filtered west neighbors; dx := 0
pub fn west_hier(node: HCoord) -> [HCoord; 2] {
    let l = node.l;
    let dh = 1 << l;
    [
        HCoord {
            l: l,
            x: node.x,
            y: node.y | dh,
        },
        HCoord {
            l: l,
            x: node.x,
            y: node.y,
        },
    ]
}
/// child filtered south neighbors; dy := 0
pub fn south_hier(node: HCoord) -> [HCoord; 2] {
    let l = node.l;
    let dh = 1 << l;
    [
        HCoord {
            l: l,
            x: node.x,
            y: node.y,
        },
        HCoord {
            l: l,
            x: node.x | dh,
            y: node.y,
        },
    ]
}
pub fn edge_neighbors(quad: &QuadTree, node: HCoord) -> Vec<HCoord> {
    // neighbor and filter need to be opposites ie (neigh east -> filter west);
    let cardinals = find_cardinals(node);
    // opposite of clockwise iteration
    let mut neighbors = Vec::new();
    let mut stack = Vec::new();
    let mut found;
    // hierarchical representation of node
    let mut h_node;
    // edge neighbor of the current node 
    let mut e_node;
    for (&cardinal, filter) in cardinals.iter().zip(EDGE_FILTERS.iter()) {
        h_node = node;
        e_node = cardinal;
        found = false;
        for lvl in node.l..quad.levels {
            e_node = transform(&e_node, lvl);
            h_node = transform(&h_node, lvl);
            if e_node == h_node {
                // information is more granular
                break;
            } else if let Some(n) = quad.information.get(&e_node) {
                if n.belief != Belief::Occupied {
                    neighbors.push(e_node);
                }
                found = true;
                break;
            }
        }
        if found {
            continue;
        }
        stack.push(cardinal);
        while let Some(p_coord) = stack.pop() {
            if let Some(n) = quad.information.get(&p_coord) {
                if n.belief == Belief::Occupied {
                    continue;
                }
                neighbors.push(p_coord);
            } else {
                stack.extend(filter(p_coord));
            }
        }
    }
    neighbors
}
