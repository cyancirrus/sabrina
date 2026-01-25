use crate::global::types::{Coord, Bounds};

pub trait SpatialSource<T> {
    fn get_bounds(&self) -> Bounds;
    fn get_cell(&self, coord: &Coord) -> T;
    fn set_cell(&mut self, coord: &Coord, object: T);
    fn raycast(&self, position: Coord, delta: Coord, max_range:usize) -> Option<Coord>;
}
