const PARTITION: usize = 32;
type Coord = (isize, isize);

pub fn encode_morton(coord: &Coord, level: isize) -> Coord {
    debug_assert!(
        coord.0 < (1 << PARTITION)
            && coord.1 < (1 << PARTITION)
            && level < (1 << isize::BITS as usize - PARTITION)
    );
    // lvl 2: 1111_1100
    let mask = !((1 << level) - 1);
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

pub fn grid_morton(coord: &Coord, level: isize) -> [Coord; 4] {
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
    println!("level: {level:?}");
    println!("(x: {}, y: {})", morton.0, morton.1,);
    println!(
        "(x: {}, y: {})",
        morton.0 & ((1 << PARTITION) - 1),
        morton.1 & ((1 << PARTITION) - 1),
    );
}
