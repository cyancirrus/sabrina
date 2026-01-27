use std::cmp::Ordering;
use std::collections::BinaryHeap;

pub type MinHeap<T> = BinaryHeap<MinNode<T>>;

/// Min Heap all costs should be non negative
///
/// #Attributes#
/// * cost
/// * cord : XY Location
#[derive(Eq, PartialEq, Debug)]
pub struct MinNode<T> {
    // All costs should be non negative
    pub cost: usize,
    pub coord: T,
}

impl<T> MinNode<T>
where
    T: Eq + PartialEq,
{
    pub fn new(cost: usize, coord: T) -> Self {
        Self { cost, coord }
    }
}

impl<T> Ord for MinNode<T>
where
    T: Eq + PartialEq,
{
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl<T> PartialOrd for MinNode<T>
where
    T: Eq + PartialEq,
{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
