pub type Coord = (isize, isize);

#[derive(Clone, Debug)]
pub struct Bounds {
    pub min_x: isize,
    pub min_y: isize,
    pub max_x: isize,
    pub max_y: isize,
}

impl Bounds {
    pub fn new(min_x: isize, min_y: isize, max_x: isize, max_y: isize) -> Self {
        Self {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }
}

#[derive(Eq, PartialEq, Copy, Clone, Debug)]
pub enum Belief {
    Free,
    Occupied,
    Unknown,
}
