use crate::global::types::{CARDINALS, SpatialMap};
use crate::global::types::{ACoord, Belief, Bounds};
use std::collections::{HashMap};

#[derive(Clone)]
pub struct Grid {
    pub information: HashMap<ACoord, Belief>,
    pub bounds: Bounds,
}
impl SpatialMap for Grid {
    type Encoded = ACoord;
    fn insert_ray(&mut self, mut pos: ACoord, hit: ACoord) {
        // beliefs not recorded are assumed free
        // handles simulation compass rose signals
        self.information.insert(hit, Belief::Occupied);
        let (dy, dx) = (hit.y  - pos.y , hit.x  - pos.x);
        let (del_y, del_x) = (dy.signum(), dx.signum());
        while pos != hit {
            pos.x += del_x;
            pos.y += del_y;
            // self.information.remove(&pos);
        }
        self.update_bounds(hit);
    }
    fn obstructed(&self, coord: ACoord) -> bool {
        // false
        self.belief(self.encode(coord)) == Belief::Occupied
    }
    fn encode(&self, coord: ACoord) -> ACoord {
        coord
    }
    fn decode(&self, node: ACoord) -> ACoord {
        node
    }
    fn distance(&self, a: ACoord, b: ACoord) -> usize {
        a.x.abs_diff(b.x).wrapping_add(a.y.abs_diff(b.y))
    }
    fn neighbors(&self, node: ACoord) -> Vec<ACoord> {
        let mut valid = Vec::new();
        for d in CARDINALS {
            let n_xy = ACoord { x: node.x + d.x, y: node.y + d.y };
            if self.belief(n_xy) != Belief::Occupied {
                valid.push(n_xy);
            }
        }
        valid
    }
    fn belief(&self, node: Self::Encoded) -> Belief {
        match self.information.get(&node) {
            Some(&belief) => belief,
            None => Belief::Free,
        }
    }
}

impl Grid {
    pub fn inspect_neighs(&self, coord: ACoord) -> Vec<ACoord> {
        let node = self.encode(coord);
        self.neighbors(node)
    }
    pub fn new() -> Self {
        let bounds = Bounds {
            min_x: isize::MAX,
            min_y: isize::MAX,
            max_x: 0,
            max_y: 0,
        };
        Self {
            information: HashMap::new(),
            bounds,
        }
    }
    pub fn update_bounds(&mut self, node: ACoord) {
        self.bounds.min_x = self.bounds.min_x.min(node.x);
        self.bounds.min_y = self.bounds.min_y.min(node.y);
        self.bounds.max_x = self.bounds.max_x.max(node.x);
        self.bounds.max_y = self.bounds.max_y.max(node.y);
    }
    pub fn belief(&self, xy: ACoord) -> Belief {
        match self.information.get(&xy) {
            Some(&belief) => belief,
            None => Belief::Unknown,
        }
    }
    pub fn raycast(&self, position: ACoord, delta: ACoord, max_range: usize) -> Option<ACoord> {
        // mock interface owning interface don't need dynamic changing env at the moment
        // RcRefcell or ArcMutex if doing pathing with multiple as extensions
        let mut n_xy = position;
        for _ in 0..max_range {
            n_xy.x = n_xy.x.wrapping_add(delta.x);
            n_xy.y = n_xy.y.wrapping_add(delta.y);
            // needs to fit wrt the underlying grid
            if self.belief(n_xy) == Belief::Occupied {
                // denormalize b/c is oracle and needs to be relative
                return Some(ACoord { 
                    x: n_xy.x.wrapping_sub(position.x),
                    y: n_xy.y.wrapping_sub(position.y),
                })
            }
        }
        None
    }
}
