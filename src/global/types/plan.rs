use crate::global::types::ACoord;
use crate::global::types::SpatialMap;

pub struct BackwardIter<'a> {
    nodes: &'a [ACoord],
    aug_index: usize,
}

pub struct ForwardIter<'a> {
    nodes: &'a [ACoord],
    length: usize,
    index: usize,
}

impl<'a> BackwardIter<'a> {
    pub fn new(nodes: &'a [ACoord]) -> Self {
        Self {
            nodes,
            aug_index: nodes.len(),
        }
    }
}

impl<'a> ForwardIter<'a> {
    pub fn new(nodes: &'a [ACoord]) -> Self {
        Self {
            nodes,
            length: nodes.len(),
            index: 0,
        }
    }
}

impl<'a> Iterator for BackwardIter<'a> {
    type Item = &'a ACoord;
    fn next(&mut self) -> Option<Self::Item> {
        if self.aug_index > 0 {
            self.aug_index -= 1;
            return Some(&self.nodes[self.aug_index]);
        }
        None
    }
}

impl<'a> Iterator for ForwardIter<'a> {
    type Item = &'a ACoord;
    fn next(&mut self) -> Option<Self::Item> {
        let index = self.index;
        if index < self.length {
            self.index += 1;
            return Some(&self.nodes[index]);
        }
        None
    }
}

/// ------------------------------------------
/// Common trait interface for plan iteration
///
/// # Example Usage
/// ```
///  # use sabrina::global::types::PlanIter;
///     fn example<P: PlanIter>(plan: &P) {
///         for p in plan.iter() { println!("{p:?}"); }
///     }
/// ```
/// ------------------------------------------
#[allow(dead_code)]
pub trait PlanIter {
    fn iter(&self) -> impl Iterator<Item = &ACoord>;
    fn nodes(&self) -> &[ACoord];
}

#[derive(Debug)]
pub struct AStarPlan {
    pub plan: Vec<ACoord>,
}

#[derive(Debug)]
pub struct DStarPlan {
    pub plan: Vec<ACoord>,
}

#[derive(Debug)]
pub struct BestFirstPlan {
    pub plan: Vec<ACoord>,
}

impl PlanIter for AStarPlan {
    fn nodes(&self) -> &[ACoord] {
        &self.plan
    }
    fn iter(&self) -> impl Iterator<Item = &ACoord> {
        BackwardIter::new(self.nodes())
    }
}

impl PlanIter for DStarPlan {
    fn nodes(&self) -> &[ACoord] {
        &self.plan
    }
    fn iter(&self) -> impl Iterator<Item = &ACoord> {
        ForwardIter::new(self.nodes())
    }
}

impl PlanIter for BestFirstPlan {
    fn nodes(&self) -> &[ACoord] {
        &self.plan
    }
    fn iter(&self) -> impl Iterator<Item = &ACoord> {
        BackwardIter::new(self.nodes())
    }
}

pub trait Planner<S: SpatialMap> {
    type Plan: PlanIter;
    fn plan(&mut self, env: &S, source: ACoord, target: ACoord) -> Option<Self::Plan>;
    fn update(&mut self, env: &S, position: ACoord, obstacle: ACoord);
}
