use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::hash::Hash;

/// Priority Queue for DStarLite
///
/// # Node :: Arguments
/// * cost_astar := min(g, rhs) + h + k;
/// * cost_dijkstra := min(g, rhs);
/// * coord := (x, y) cordinate
pub type KeyHeap<T> = BinaryHeap<KeyNode<T>>;

#[derive(Eq, PartialEq, Debug, Copy, Clone)]
pub struct StarKey {
    pub cost_astar: usize,
    pub cost_dijkstra: usize,
}

impl StarKey {
    pub fn new(g: usize, rhs: usize, h: usize, k: usize) -> Self {
        let cost_dijkstra = g.min(rhs);
        Self {
            cost_astar: cost_dijkstra.wrapping_add(h).wrapping_add(k),
            cost_dijkstra,
        }
    }
}

impl Ord for StarKey {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost_astar
            .cmp(&self.cost_astar)
            .then_with(|| other.cost_dijkstra.cmp(&self.cost_dijkstra))
    }
}

impl PartialOrd for StarKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
/// Represents the underlying DStarLite Estimates
///
/// # Arguments
/// * cost_star := min(g, rhs) + h;
/// * cost_dijsktra := min(g, rhs);
/// * coord := (x, y) cordinate
#[derive(Eq, PartialEq, Debug)]
pub struct KeyNode<T> {
    // All costs should be non negative
    pub star_key: StarKey, // min(g), rhs + h + k;
    pub coord: T,
}

/// Key from the paper: g, rhs+h
impl<T> KeyNode<T>
where
    T: Hash,
{
    pub fn new(coord: T, g: usize, rhs: usize, h: usize, k: usize) -> Self {
        let cost_dijkstra = g.min(rhs);
        let cost_astar = cost_dijkstra.saturating_add(h).saturating_add(k);
        Self {
            star_key: StarKey {
                cost_astar,
                cost_dijkstra,
            },
            coord,
        }
    }
}

impl<T> Ord for KeyNode<T>
where
    T: Hash + Eq,
{
    fn cmp(&self, other: &Self) -> Ordering {
        self.star_key.cmp(&other.star_key)
    }
}

impl<T> PartialOrd for KeyNode<T>
where
    T: Hash + Eq,
{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
