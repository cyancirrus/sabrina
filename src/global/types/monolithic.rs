pub type Coord = (usize, usize);
#[derive(Clone, Debug)]
pub struct Bounds {
    pub min_x: isize,
    pub min_y: isize,
    pub max_x: isize,
    pub max_y: isize,
}

#[derive(Eq, PartialEq, Copy, Clone, Debug)]
pub enum Belief {
    Free,
    Occupied,
    Unknown,
}

#[derive(Eq, PartialEq, Debug)]
pub enum Status {
    Enroute,
    Blocked,
    Impossible,
    Complete,
}
