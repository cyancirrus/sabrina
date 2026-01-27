use crate::environment::quad::QuadTree;
use crate::global::consts::PARTITION;
use crate::global::types::{Belief, Coord,};
use crate::hierarchy::encoding::encode_hier;

// // Observation Logic (Unknown \(\rightarrow \) Free/Occupied).LU Pivoting (Numerical insurance)
// // Multi-ray LiDAR & Planner Implementation.
// // Hestereses or defered clean up
// // Consdier implementing a jump iter

// TODO: Think through whether the boundary case exists where we aren't surrounded by wall

pub fn find_cardinals(m_coord: Coord) -> [Coord; 4] {
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
pub fn east_hier(hier: Coord) -> [Coord; 2] {
    // child filtered east neighbors; dx := 1
    let level = (hier.0 >> PARTITION) - 1;
    [
        (
            (hier.0 - (1 << PARTITION)) | 1 << level,
            (hier.1 - (1 << PARTITION)),
        ),
        (
            (hier.0 - (1 << PARTITION)) | 1 << level,
            (hier.1 - (1 << PARTITION)) | 1 << level,
        ),
    ]
}
pub fn north_hier(hier: Coord) -> [Coord; 2] {
    // child filtered west neighbors; dy := 1
    let level = (hier.0 >> PARTITION) - 1;
    [
        (
            (hier.0 - (1 << PARTITION)) | 1 << level,
            (hier.1 - (1 << PARTITION)) | 1 << level,
        ),
        (
            (hier.0 - (1 << PARTITION)),
            (hier.1 - (1 << PARTITION)) | 1 << level,
        ),
    ]
}
pub fn west_hier(hier: Coord) -> [Coord; 2] {
    // child filtered west neighbors; dx := 0
    let level = (hier.0 >> PARTITION) - 1;
    [
        (
            (hier.0 - (1 << PARTITION)),
            (hier.1 - (1 << PARTITION)) | 1 << level,
        ),
        ((hier.0 - (1 << PARTITION)), (hier.1 - (1 << PARTITION))),
    ]
}
pub fn south_hier(hier: Coord) -> [Coord; 2] {
    // child filtered south neighbors; dy := 0
    let level = (hier.0 >> PARTITION) - 1;
    [
        ((hier.0 - (1 << PARTITION)), (hier.1 - (1 << PARTITION))),
        (
            (hier.0 - (1 << PARTITION)) | 1 << level,
            (hier.1 - (1 << PARTITION)),
        ),
    ]
}

pub fn edge_neighbors(quad: &QuadTree, m_coord: Coord) -> Vec<Coord> {
    // neighbor and filter need to be opposites ie (neigh east -> filter west);
    let cardinals = find_cardinals(m_coord);
    // opposite of clockwise iteration
    let filters = [west_hier, south_hier, east_hier, north_hier];
    let level = m_coord.0 >> PARTITION;
    let mut neighbors = Vec::new();
    let mut stack = Vec::new();
    let mut found;
    for (cardinal, filter) in cardinals.iter().zip(filters.iter()) {
        found = false;
        for lvl in level..quad.levels {
            let p_coord = encode_hier(&cardinal, lvl);
            if let Some(n) = quad.information.get(&p_coord) {
                if n.belief != Belief::Occupied {
                    neighbors.push(p_coord);
                }
                found = true;
                break;
            } else if encode_hier(&m_coord, lvl) == p_coord {
                break;
            }
        }
        if found {
            continue;
        }
        stack.push(*cardinal);
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
