use crate::global::types::Coord;

pub struct BackwardIter<'a> {
    nodes: &'a [Coord],
    aug_index: usize,
}

pub struct ForwardIter<'a> {
    nodes: &'a [Coord],
    length: usize,
    index: usize,
}

impl<'a> BackwardIter<'a> {
    pub fn new(nodes: &'a [Coord]) -> Self {
        Self {
            nodes,
            aug_index: nodes.len(),
        }
    }
}

impl<'a> ForwardIter<'a> {
    pub fn new(nodes: &'a [Coord]) -> Self {
        Self {
            nodes,
            length: nodes.len(),
            index: 0,
        }
    }
}

impl<'a> Iterator for BackwardIter<'a> {
    type Item = &'a Coord;
    fn next(&mut self) -> Option<Self::Item> {
        if self.aug_index > 0 {
            self.aug_index -= 1;
            return Some(&self.nodes[self.aug_index]);
        }
        None
    }
}

impl<'a> Iterator for ForwardIter<'a> {
    type Item = &'a Coord;
    fn next(&mut self) -> Option<Self::Item> {
        let index = self.index;
        if index < self.length {
            self.index += 1;
            return Some(&self.nodes[index]);
        }
        None
    }
}

#[allow(dead_code)]
pub trait PlanIter {
    fn iter(&self) -> impl Iterator<Item = &Coord>;
    fn nodes(&self) -> &[Coord];
}

pub struct AStarPlan {
    pub plan: Vec<Coord>,
}

pub struct DStarPlan {
    pub plan: Vec<Coord>,
}

impl PlanIter for AStarPlan {
    fn nodes(&self) -> &[Coord] {
        &self.plan
    }
    fn iter(&self) -> impl Iterator<Item = &Coord> {
        BackwardIter::new(self.nodes())
    }
}

impl PlanIter for DStarPlan {
    fn nodes(&self) -> &[Coord] {
        &self.plan
    }
    fn iter(&self) -> impl Iterator<Item = &Coord> {
        ForwardIter::new(self.nodes())
    }
}

