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
    pub cost_astar: usize, // min(g, rhs) + h;
    pub cost_dijkstra: usize, // min(g, rhs)
    pub coord: Coord,
}

impl KeyNode {
    pub fn new(coord: Coord, g: usize, rhs: usize, h: usize) -> Self {
        let cost_dijkstra = g.min(rhs);
        let cost_astar = cost_dijkstra.saturating_add(h);
        Self {
            cost_astar,
            cost_dijkstra,
            coord,
        }
    }
}

impl Ord for KeyNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost_astar
            .cmp(&self.cost_astar)
            .then_with(|| other.cost_dijkstra.cmp(&self.cost_dijkstra))
    }
}

impl PartialOrd for KeyNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// Used for weighted descent
#[derive(Eq, PartialEq, Debug)]
pub struct HeurMinNode {
    // All costs should be non negative
    pub coord: Coord,
    pub cost: usize,
    pub incurred: usize
}

impl HeurMinNode {
    pub fn new(cost: usize, coord: Coord) -> Self {
        Self { cost, coord, incurred:0 }
    }
}

impl Ord for HeurMinNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl PartialOrd for HeurMinNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
