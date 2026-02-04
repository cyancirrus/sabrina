use crate::global::types::{ACoord, HCoord};

// // Hestereses or defered clean up
// // Consdier implementing a jump iter

// TODO: Think through whether the boundary case exists where we aren't surrounded by wall

// // Observation Logic (Unknown \(\rightarrow \) Free/Occupied).LU Pivoting (Numerical insurance)
// // Multi-ray LiDAR & Planner Implementation.
// // Hestereses or defered clean up
// // Consdier implementing a jump iter
pub fn point(h_coord: HCoord) -> ACoord {
    // An interface for retrieving purely for retrieving distance
    // retrieves the centroid scaled by two in order to prevent half-integers
    let center = 1 << h_coord.l;
    ACoord {
        x: h_coord.x + center,
        y: h_coord.y + center,
    }
}

pub fn distance(a: HCoord, b: HCoord) -> usize {
    // distance between source-centroid and target-centroid
    let (a, b) = (point(a), point(b));
    a.x.abs_diff(b.x) + a.y.abs_diff(b.y)
}

/// floating point representation of centroid of hierarchy
pub fn decode_hier(coord: HCoord) -> (f32, f32) {
    let dp = point(coord);
    ((dp.x - 1) as f32 / 2.0, (dp.x - 1) as f32 / 2.0)
}

pub fn encode(coord: ACoord, level: usize) -> HCoord {
    let mask = !((1 << level) - 1);
    HCoord {
        l: level,
        x: coord.x & mask,
        y: coord.y & mask,
    }
}

pub fn transform(coord: &HCoord, level: usize) -> HCoord {
    if level < coord.l {
        return *coord;
    }
    let mask = !((1 << level) - 1);
    HCoord {
        l: level,
        x: coord.x & mask,
        y: coord.y & mask,
    }
}

pub fn child_hier(hier: &HCoord) -> [HCoord; 4] {
    // level of the child
    let l = hier.l - 1;
    // separation of the grid
    let dh = 1 << l;
    [
        HCoord {
            l: l,
            x: hier.x,
            y: hier.y,
        },
        HCoord {
            l: l,
            x: hier.x + dh,
            y: hier.y,
        },
        HCoord {
            l: l,
            x: hier.x,
            y: hier.y + dh,
        },
        HCoord {
            l: l,
            x: hier.x + dh,
            y: hier.y + dh,
        },
    ]
}

/// Encode a node with a new level of the hierarchy
pub fn grid_hier(node: &ACoord, level: usize) -> [HCoord; 4] {
    // last bit of this level cleared as well, lvl + 1
    let dh = 1 << level;
    let mask = !((1 << (level + 1)) - 1);
    let (x, y) = (node.x & mask, node.y & mask);
    // clockwise navigation through grid
    [
        HCoord {
            l: level,
            x: x,
            y: y,
        },
        HCoord {
            l: level,
            x: x + dh,
            y: y,
        },
        HCoord {
            l: level,
            x: x,
            y: y + dh,
        },
        HCoord {
            l: level,
            x: x + dh,
            y: y + dh,
        },
    ]
}
