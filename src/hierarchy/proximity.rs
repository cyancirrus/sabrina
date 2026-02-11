use crate::environment::quad::QuadTree;
use crate::global::types::{Belief, HCoord};
use crate::hierarchy::encoding::transform;

// // Observation Logic (Unknown \(\rightarrow \) Free/Occupied).LU Pivoting (Numerical insurance)
// // Multi-ray LiDAR & Planner Implementation.
// // Hestereses or defered clean up
// // Consdier implementing a jump iter

// TODO: Think through whether the boundary case exists where we aren't surrounded by wall

/// Finds north south east west not the quadgrid
const EDGE_FILTERS: [fn(HCoord) -> [HCoord; 2]; 4] = [west_hier, south_hier, east_hier, north_hier];

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
            x: node.x,
            y: node.y - dh,
        },
    ]
}
/// child nodes filter east neighbors; dx := 1
pub fn east_hier(node: HCoord) -> [HCoord; 2] {
    let l = node.l - 1;
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
    let l = node.l - 1;
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
    let l = node.l - 1;
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
    let l = node.l - 1;
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
                // NOTE: this is just an optimization
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
            } else if p_coord.l > 0 {
                stack.extend(filter(p_coord));
            } else {
                neighbors.push(p_coord);
            }
        }
    }
    neighbors
}

pub fn grid_siblings(node: &HCoord) -> [HCoord; 4] {
    // goes in clockwise direction
    // [bl, br, tr, tl]
    let h = 1 << node.l;
    [
        HCoord {
            l: node.l,
            x: node.x & !h,
            y: node.y & !h,
        },
        HCoord {
            l: node.l,
            x: node.x | h,
            y: node.y & !h,
        },
        HCoord {
            l: node.l,
            x: node.x | h,
            y: node.y | h,
        },
        HCoord {
            l: node.l,
            x: node.x & !h,
            y: node.y | h,
        },
    ]
}

pub fn grid_children(node: &HCoord) -> [HCoord; 4] {
    // goes in clockwise direction
    // [bl, br, tr, tl]
    let l = node.l - 1;
    let h = 1 << l;
    [
        HCoord {
            l: l,
            x: node.x & !h,
            y: node.y & !h,
        },
        HCoord {
            l: l,
            x: node.x | h,
            y: node.y & !h,
        },
        HCoord {
            l: l,
            x: node.x | h,
            y: node.y | h,
        },
        HCoord {
            l: l,
            x: node.x & !h,
            y: node.y | h,
        },
    ]
}
#[cfg(test)]
mod test_grid_siblings {
    use crate::global::types::HCoord;
    use crate::hierarchy::proximity::grid_siblings;
    #[test]
    fn test_positive_sibblings() {
        let x = HCoord { l: 0, x: 0, y: 0 };
        let eresult = [
            HCoord { l: 0, x: 0, y: 0 },
            HCoord { l: 0, x: 1, y: 0 },
            HCoord { l: 0, x: 1, y: 1 },
            HCoord { l: 0, x: 0, y: 1 },
        ];
        assert_eq!(eresult, grid_siblings(&x));
        let x = HCoord { l: 0, x: 1, y: 1 };
        assert_eq!(eresult, grid_siblings(&x));
        let x = HCoord { l: 2, x: 0, y: 0 };
        let eresult = [
            HCoord { l: 2, x: 0, y: 0 },
            HCoord { l: 2, x: 4, y: 0 },
            HCoord { l: 2, x: 4, y: 4 },
            HCoord { l: 2, x: 0, y: 4 },
        ];
        assert_eq!(eresult, grid_siblings(&x));
    }
    #[test]
    fn test_negative_sibblings() {
        let x = HCoord { l: 0, x: -1, y: -1 };
        let eresult = [
            HCoord { l: 0, x: -2, y: -2 },
            HCoord { l: 0, x: -1, y: -2 },
            HCoord { l: 0, x: -1, y: -1 },
            HCoord { l: 0, x: -2, y: -1 },
        ];
        assert_eq!(eresult, grid_siblings(&x));
        let x = HCoord { l: 0, x: -2, y: -1 };
        assert_eq!(eresult, grid_siblings(&x));
        let x = HCoord { l: 2, x: -1, y: -1 };
        let eresult = [
            HCoord { l: 2, x: -5, y: -5 },
            HCoord { l: 2, x: -1, y: -5 },
            HCoord { l: 2, x: -1, y: -1 },
            HCoord { l: 2, x: -5, y: -1 },
        ];
        assert_eq!(eresult, grid_siblings(&x));
    }
}
