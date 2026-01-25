use crate::algo::a_star::point;
use crate::global::types::Coord;
const PARTITION: usize = 32;

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
