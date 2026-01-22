use std::cmp::Ordering;
pub type Coord = (usize, usize);

#[derive(Clone, Debug)]
pub struct Bounds {
    pub min_x: usize,
    pub min_y: usize,
    pub max_x: usize,
    pub max_y: usize,
}

impl Bounds {
    pub fn new(min_x: usize, min_y: usize, max_x: usize, max_y: usize) -> Self {
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

#[derive(Eq, PartialEq, Debug)]
pub struct MinNode {
    // All costs should be non negative
    pub cost: usize,
    pub coord: Coord,
}

impl MinNode {
    pub fn new(cost: usize, coord: Coord) -> Self {
        Self { cost, coord }
    }
}

impl Ord for MinNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl PartialOrd for MinNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Eq, PartialEq, Debug)]
pub struct KeyNode {
    // All costs should be non negative
    pub cost_astar: usize,
    pub cost_dfs: usize,
    pub coord: Coord,
}

impl KeyNode {
    pub fn new(coord: Coord, g: usize, rhs: usize, h: usize) -> Self {
        let cost_dfs = g.min(rhs);
        let cost_astar = cost_dfs.saturating_add(h);
        Self {
            cost_astar,
            cost_dfs,
            coord,
        }
    }
}

impl Ord for KeyNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost_astar
            .cmp(&self.cost_astar)
            .then_with(|| other.cost_dfs.cmp(&self.cost_dfs))
    }
}

impl PartialOrd for KeyNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
