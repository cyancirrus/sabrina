use crate::global::consts::PARTITION;
use crate::global::types::Coord;

// // Observation Logic (Unknown \(\rightarrow \) Free/Occupied).LU Pivoting (Numerical insurance)
// // Multi-ray LiDAR & Planner Implementation.
// // Hestereses or defered clean up
// // Consdier implementing a jump iter

// TODO: Think through whether the boundary case exists where we aren't surrounded by wall

pub fn point(m_coord: Coord) -> Coord {
    // An interface for retrieving purely for retrieving distance
    // retrieves the centroid scaled by two in order to prevent half-integers
    let level = m_coord.0 >> PARTITION;
    let mask = (1 << PARTITION) - 1;
    (
        ((m_coord.0 & mask) << 1) + (1 << level),
        ((m_coord.1 & mask) << 1) + (1 << level),
    )
}
pub fn centroid_estimate(sm_coord: Coord, tm_coord: Coord) -> usize {
    // distance between source-centroid and target-centroid
    let (s_centr, t_centr) = (point(sm_coord), point(tm_coord));
    s_centr.0.abs_diff(t_centr.0) + s_centr.1.abs_diff(t_centr.1)
}

pub fn decode_hier(coord: Coord) -> (f32, f32) {
    let dp = point(coord);
    ((dp.0 - 1) as f32 / 2.0, (dp.0 - 1) as f32 / 2.0)
}

pub fn encode_hier(coord: &Coord, level: usize) -> Coord {
    debug_assert!(level < (1 << usize::BITS as usize - PARTITION));
    let mask = ((1 << PARTITION) - 1) & !((1 << level) - 1);
    (
        (coord.0 & mask) | (level << PARTITION),
        (coord.1 & mask) | (level << PARTITION),
    )
}

pub fn child_hier(hier: &Coord) -> [Coord; 4] {
    // level of the child
    let level = (hier.0 >> PARTITION) - 1;
    [
        ((hier.0 - (1 << PARTITION)), (hier.1 - (1 << PARTITION))),
        (
            (hier.0 - (1 << PARTITION)) | 1 << level,
            (hier.1 - (1 << PARTITION)),
        ),
        (
            (hier.0 - (1 << PARTITION)),
            (hier.1 - (1 << PARTITION)) | 1 << level,
        ),
        (
            (hier.0 - (1 << PARTITION)) | 1 << level,
            (hier.1 - (1 << PARTITION)) | 1 << level,
        ),
    ]
}

pub fn grid_hier(coord: &Coord, level: usize) -> [Coord; 4] {
    // last bit of this level cleared as well, lvl + 1
    let mask = !((1 << (level + 1)) - 1);
    // clockwise navigation through grid
    [
        (
            (coord.0 & mask) | (level << PARTITION),
            (coord.1 & mask) | (level << PARTITION),
        ),
        (
            (coord.0 & mask) | (level << PARTITION) | 1 << level,
            (coord.1 & mask) | (level << PARTITION),
        ),
        (
            (coord.0 & mask) | (level << PARTITION),
            (coord.1 & mask) | (level << PARTITION) | 1 << level,
        ),
        (
            (coord.0 & mask) | (level << PARTITION) | 1 << level,
            (coord.1 & mask) | (level << PARTITION) | 1 << level,
        ),
    ]
}

pub fn print_hier(hier: &Coord) {
    let level = hier.0 >> PARTITION;
    println!("-------------");
    println!("level: {level:?}");
    println!("or: (x: {}, y: {})", hier.0, hier.1,);
    println!(
        "hr: (x: {}, y: {})",
        hier.0 & ((1 << PARTITION) - 1),
        hier.1 & ((1 << PARTITION) - 1),
    );
    println!("-------------");
}
