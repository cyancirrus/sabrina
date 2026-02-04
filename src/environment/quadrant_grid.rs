use sabrina::global::types::Belief;
use sabrina::global::types::SpatialMap;

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
#[derive(Eq, PartialEq, Copy, Clone)]
struct ACoord {
    x: isize,
    y: isize,
}
/// Point for the quadrant
///
/// # Attributes #
/// * q: quadrant
/// * x: offset index
/// * y: offset index
struct QPoint {
    q: usize,
    x: usize,
    y: usize,
}
type Storage = QPoint;
type Encoded = ACoord;




impl QuadrantGrid {
    fn new() -> Self {
        Self {
            q: [
                Vec::with_capacity(8),
                Vec::with_capacity(8),
                Vec::with_capacity(8),
                Vec::with_capacity(8),
            ],
        }
    }
    fn insert_ray(&mut self, mut pos:ACoord, hit:ACoord) {
        // beliefs not recorded are assumed unknown
        // handles simulation compass rose signals
        let (dy, dx) = (hit.y  - pos.y , hit.x  - pos.x);
        let (del_y, del_x) = (dy.signum(), dx.signum());
        while pos != hit {
            pos.x += del_x;
            pos.y += del_y;
            self.update_belief(hit, Belief::Occupied);
        }
        self.update_belief(hit, Belief::Occupied);
    }
    fn update_belief(&mut self, coord:ACoord, belief:Belief) {
        let s= self.transform(coord);
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
    fn transform(&self, coord: ACoord) -> Storage {
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
    fn untransform(&self, node: Storage) -> ACoord {
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
    fn encode(&self, coord: Encoded) -> ACoord {
        coord
    }
    fn decode(&self, coord: Encoded) -> ACoord {
        coord
    }
    fn belief(&self, node: Encoded) -> Belief {
        let store = self.transform(node);
        if self.q[store.q].len() <= store.x || self.q[store.q][store.x].len() <= store.y {
            Belief::Unknown
        } else {
            self.q[store.q][store.x][store.y]
        }
    }
    fn distance(&self, a:Encoded, b:Encoded) -> usize {
        a.x.abs_diff(b.x) + a.y.abs_diff(b.y)
    }
    fn obstructed(&self, node: Encoded) -> bool {
        match self.belief(node) {
            Belief::Occupied => true,
            _ => false
        }
    }
    fn neighbors(&self, node:Encoded) -> Vec<Encoded> {
        vec![
            Encoded {
                x: node.x + 1,
                y: node.y
            },
            Encoded {
                x: node.x,
                y: node.y + 1
            },
            Encoded {
                x: node.x - 1,
                y: node.y
            },
            Encoded {
                x: node.x ,
                y: node.y - 1,
            },

        ]

    }
}

impl QuadrantGrid {}

fn main() {}
