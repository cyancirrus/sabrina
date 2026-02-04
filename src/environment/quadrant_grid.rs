use crate::global::types::{ACoord, Belief, SpatialMap};

/// Ordering reflects quadrants in standard euclidean
///
/// Q1 :: x >= 0, y >= 0;
/// Q2 :: x <  0, y  > 0;
/// Q3 :: x <= 0, y <= 0;
/// Q3 :: x >  0, y <  0;
///
/// quadrants organized by x, y
pub struct QuadrantGrid {
    q: [Vec<Vec<Belief>>; 4],
}
/// Point for the quadrant
///
/// # Attributes #
/// * q: quadrant
/// * x: offset index
/// * y: offset index
pub struct QPoint {
    q: usize,
    x: usize,
    y: usize,
}
type Storage = QPoint;

impl SpatialMap for QuadrantGrid {
    type Encoded = ACoord;
    fn encode(&self, coord: ACoord) -> Self::Encoded {
        coord
    }
    fn decode(&self, coord: Self::Encoded) -> ACoord {
        coord
    }
    fn belief(&self, node: Self::Encoded) -> Belief {
        let store = self.transform(node);
        if self.q[store.q].len() <= store.x || self.q[store.q][store.x].len() <= store.y {
            Belief::Unknown
        } else {
            self.q[store.q][store.x][store.y]
        }
    }
    fn distance(&self, a: Self::Encoded, b: Self::Encoded) -> usize {
        a.x.abs_diff(b.x) + a.y.abs_diff(b.y)
    }
    fn obstructed(&self, coord: ACoord) -> bool {
        match self.belief(coord) {
            Belief::Occupied => true,
            _ => false,
        }
    }
    fn neighbors(&self, node: Self::Encoded) -> Vec<Self::Encoded> {
        vec![
            Self::Encoded {
                x: node.x + 1,
                y: node.y,
            },
            Self::Encoded {
                x: node.x,
                y: node.y + 1,
            },
            Self::Encoded {
                x: node.x - 1,
                y: node.y,
            },
            Self::Encoded {
                x: node.x,
                y: node.y - 1,
            },
        ]
    }
    fn insert_ray(&mut self, mut pos: ACoord, hit: ACoord) {
        // beliefs not recorded are assumed unknown
        // handles simulation compass rose signals
        let (dy, dx) = (hit.y - pos.y, hit.x - pos.x);
        let (del_y, del_x) = (dy.signum(), dx.signum());
        pos.x += del_x;
        pos.y += del_y;
        while pos != hit {
            self.update_belief(hit, Belief::Free);
            pos.x += del_x;
            pos.y += del_y;
        }
        self.update_belief(hit, Belief::Occupied);
    }
}

impl QuadrantGrid {
    pub fn new() -> Self {
        Self {
            q: [
                Vec::with_capacity(8),
                Vec::with_capacity(8),
                Vec::with_capacity(8),
                Vec::with_capacity(8),
            ],
        }
    }
    fn update_belief(&mut self, coord: ACoord, belief: Belief) {
        let s = self.transform(coord);
        let m = self.q[s.q].len();
        for _ in s.x..=m {
            self.q[s.q].push(Vec::new());
        }
        let n = self.q[s.q][s.x].len();
        for _ in s.y..=n {
            self.q[s.q][s.x].push(Belief::Unknown);
        }
        self.q[s.q][s.x][s.y] = belief;
    }
    pub fn transform(&self, coord: ACoord) -> Storage {
        let (mut x, mut y) = (coord.x.abs(), coord.y.abs());
        let q = match (coord.x >= 0, coord.y >= 0) {
            (true, true) => 0,
            (false, true) => 1,
            (false, false) => 2,
            (true, false) => 3,
        };
        if q == 1 || q == 3 {
            (x, y) = (x - 1, y - 1);
        }
        Storage {
            q,
            x: x as usize,
            y: y as usize,
        }
    }
    pub fn untransform(&self, node: Storage) -> ACoord {
        // Valid for quadrant 0;
        let mut acoord = ACoord {
            x: node.x as isize,
            y: node.y as isize,
        };
        match node.q {
            1 => {
                acoord.x = -1 - acoord.x;
                acoord.y += 1;
            }
            2 => {
                acoord.x = -acoord.x;
                acoord.y = -acoord.y;
            }
            3 => {
                acoord.x += 1;
                acoord.y = -1 - acoord.y;
            }
            _ => {}
        };
        acoord
    }
}
