#![allow(unused)]
use crate::global::consts::GRID_OFFSET;
use crate::global::types::SpatialMap;
use crate::global::types::{Belief, Bounds, Coord};
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fmt;
use std::fs;

type TCoord = (usize, usize);

#[derive(Clone)]
pub struct Grid {
    pub information: HashMap<Coord, Belief>,
    pub seen: Bounds,
    pub bounds: Bounds,
}
pub fn translate(xy: Coord) -> Coord {
    (
        xy.0.wrapping_add(GRID_OFFSET),
        xy.1.wrapping_add(GRID_OFFSET),
    )
}

impl SpatialMap for Grid {
    type Encoded = TCoord;
    fn insert_ray(&mut self, pos: Coord, hit: Coord) {
        // beliefs not recorded are assumed free
        // handles simulation compass rose signals
        let (dy, dx) = (
            hit.1 as isize - pos.1 as isize,
            hit.0 as isize - pos.0 as isize,
        );
        let (del_y, del_x) = (dy.signum(), dx.signum());
        let mut e_pos = self.encode(pos);
        let e_hit = self.encode(hit);
        self.update_bounds(e_hit);
        while e_pos != e_hit {
            e_pos = (
                e_pos.0.wrapping_add(del_x as usize),
                e_pos.1.wrapping_add(del_y as usize),
            );
            self.information.remove(&e_pos);
        }
        self.information.insert(e_hit, Belief::Occupied);
    }
    fn obstructed(&self, coord: Coord) -> bool {
        // false
        *self.belief(self.encode(coord)) == Belief::Occupied
    }
    fn encode(&self, coord: Coord) -> TCoord {
        (
            coord.0.wrapping_add(GRID_OFFSET),
            coord.1.wrapping_add(GRID_OFFSET),
        )
    }
    fn decode(&self, node: TCoord) -> Coord {
        (
            node.0.wrapping_sub(GRID_OFFSET),
            node.1.wrapping_sub(GRID_OFFSET),
        )
    }
    fn distance(&self, a: TCoord, b: TCoord) -> usize {
        a.0.abs_diff(b.0) + a.1.abs_diff(b.1)
    }
    fn neighbors(&self, node: TCoord) -> Vec<TCoord> {
        let mut valid = Vec::new();
        let delta = [(1, 0), (0, 1), (!0, 0), (0, !0)];
        for (dx, dy) in delta {
            // let n_xy = (node.0 + dx, node.1 + dy);
            let n_xy = (node.0.wrapping_add(dx), node.1.wrapping_add(dy));
            if *self.belief(n_xy) != Belief::Occupied {
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
    pub fn new() -> Self {
        let seen = Bounds {
            min_x: usize::MAX,
            min_y: usize::MAX,
            max_x: 0,
            max_y: 0,
        };
        let bounds = Bounds {
            min_x: 0,
            min_y: 0,
            max_x: 4,
            max_y: 4,
        };
        Self {
            information: HashMap::new(),
            bounds,
            seen,
        }
    }
    pub fn update_bounds(&mut self, node: TCoord) {
        self.seen.min_x = self.seen.min_x.min(node.0);
        self.seen.min_y = self.seen.min_y.min(node.1);
        self.seen.max_x = self.seen.max_x.max(node.0);
        self.seen.max_y = self.seen.max_y.max(node.1);
        self.bounds.min_x = self.bounds.min_x.min(node.0);
        self.bounds.min_y = self.bounds.min_y.min(node.1);
        self.bounds.max_x = self.bounds.max_x.max(node.0);
        self.bounds.max_y = self.bounds.max_y.max(node.1);
    }
    pub fn path_clear(&self, xy: Coord) -> bool {
        !self.information.contains_key(&translate(xy))
    }
    pub fn belief(&self, xy: Coord) -> &Belief {
        match self.information.get(&xy) {
            Some(belief) => belief,
            None => &Belief::Unknown,
        }
    }
    pub fn raycast(&self, position: Coord, delta: Coord, max_range: usize) -> Option<Coord> {
        // mock interface owning interface don't need dynamic changing env at the moment
        // RcRefcell or ArcMutex if doing pathing with multiple as extensions
        let mut n_xy = position;
        for _ in 1..max_range {
            n_xy.0 = n_xy.0.wrapping_add(delta.0);
            n_xy.1 = n_xy.1.wrapping_add(delta.1);
            // needs to fit wrt the underlying grid
            if !self.path_clear(n_xy) {
                // denomralize b/c is oracle and needs to be relative
                let denorm_xy = (
                    n_xy.0.wrapping_sub(position.0),
                    n_xy.1.wrapping_sub(position.1),
                );
                return Some(denorm_xy);
            }
        }
        None
    }
}
