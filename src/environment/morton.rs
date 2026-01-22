use crate::global::types::Coord;
const PARTITION: usize = 32;

pub fn encode_morton(coord: &Coord, level: usize) -> Coord {
    debug_assert!(level < (1 << usize::BITS as usize - PARTITION));
    let mask = ((1 << PARTITION) - 1) & !((1 << level) - 1);
    (
        (coord.0 & mask) | (level << PARTITION),
        (coord.1 & mask) | (level << PARTITION),
    )
}

pub fn child_morton(morton: &Coord) -> [Coord; 4] {
    // level of the child
    let level = (morton.0 >> PARTITION) - 1;
    [
        ((morton.0 - (1 << PARTITION)), (morton.1 - (1 << PARTITION))),
        (
            (morton.0 - (1 << PARTITION)) | 1 << level,
            (morton.1 - (1 << PARTITION)),
        ),
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

pub fn grid_morton(coord: &Coord, level: usize) -> [Coord; 4] {
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

pub fn print_morton(morton: &Coord) {
    let level = morton.0 >> PARTITION;
    println!("-------------");
    println!("level: {level:?}");
    println!("or: (x: {}, y: {})", morton.0, morton.1,);
    println!(
        "hr: (x: {}, y: {})",
        morton.0 & ((1 << PARTITION) - 1),
        morton.1 & ((1 << PARTITION) - 1),
    );
    println!("-------------");
}
