#![allow(unused)]
use sabrina::algo::a_star::{astar, centroid_estimate, edge_neighbors, point};
use sabrina::algo::d_star::{LazyPQueue, Star, dstar_lite};
use sabrina::environment::grid::Grid;
use sabrina::environment::info::reconstruct;
use sabrina::environment::hier::{
    child_hier, decode_hier, encode_hier, grid_hier, print_hier,
};
use sabrina::environment::quad::{QuadTree, QuadNode};
use sabrina::global::consts::{LEVELS, PARTITION};
use sabrina::global::types::{Belief, Bounds, Coord, KeyNode, MinNode};
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::read_grid;
use sabrina::parser::quad::read_quad;
use sabrina::sensor::lidar::Lidar;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::mem;
use std::time::Instant;
// use std::slice::Iter;

/// Interface to unite astar and dstarlite traversal
/// #Members#
/// * Forward := planner creates nodes from source -> target,
/// * Backward := planner creates nodes from target -> source,

pub struct ForwardIter<'a> {
    nodes: &'a [Coord],
    aug_index: usize,
}

pub struct BackwardIter<'a> {
    nodes: &'a [Coord],
    length: usize,
    index: usize,
}

impl <'a> ForwardIter<'a> {
    pub fn new(nodes:&'a [Coord]) -> Self {
        Self {
            nodes,
            aug_index:nodes.len(),
        }
    }
}

impl <'a> BackwardIter<'a> {
    pub fn new(nodes:&'a [Coord]) -> Self {
        Self {
            nodes,
            length: nodes.len(),
            index: 0,
        }
    }
}

impl <'a> Iterator for ForwardIter<'a> {
    type Item = &'a Coord;
    fn next(&mut self) -> Option<Self::Item> {
        if self.aug_index > 0 {
            self.aug_index -= 1;
            return Some(&self.nodes[self.aug_index])
        }
        None
    }
}

impl <'a> Iterator for BackwardIter<'a> {
    type Item = &'a Coord;
    fn next(&mut self) -> Option<Self::Item> {
        let index = self.index;
        if index < self.length {
            self.index += 1;
            return Some(&self.nodes[index])
        }
        None
    }
}

trait PlanIter {
    fn iter(&self) -> impl Iterator<Item = &Coord>;
    fn nodes(&self) -> &[Coord];
}


pub struct AStarPlan {
    plan: Vec<Coord>,
}

pub struct DStarPlan {
    plan: Vec<Coord>,
}

impl PlanIter for AStarPlan {
    fn nodes(&self) -> &[Coord] {
        &self.plan
    }
    fn iter(&self) -> impl Iterator<Item = &Coord> {
        ForwardIter::new(self.nodes()) 
    }
}

impl PlanIter for DStarPlan {
    fn nodes(&self) -> &[Coord] {
        &self.plan
    }
    fn iter(&self) -> impl Iterator<Item = &Coord> {
        BackwardIter::new(self.nodes()) 
    }
}


fn run_plan<P: PlanIter>(plan: &P) {
    for c in plan.iter() {
        println!("{c:?}");
    }

}




// pub trait Plan {
    // fn traversal(&self) -> Traversal;
    // fn nodes(&self) -> &[Coord];
    // fn iter(&self) -> impl Iterator<Item = &Coord> {
    //     match self.traversal() {
    //         Traversal::Forward => self.nodes().iter(),
    //         Traversal::Backward => self.nodes().iter().rev(),
    //     }
    // }
// }


// pub struct DStar {
//     update_queue: LazyPQueue,
//     star: Star,
//     plan: Vec<Coord>
// }

// impl <'a> Plan for AStar<'a> {
//     fn nodes(&self) -> &[Coord] {
//         &self.plan
//     }
// }

// impl <'s> Plan for DStar<'s> {
//     fn nodes(&self) -> &[Coord] {
//         &self.plan
//     }
// }

// // i:= iterator, s:= *'s internal
// impl <'i, 's> IntoIterator for &'i AStar<'s> {
//     type Item = &'i Coord;
//     type IntoIter = Iter<'i, Coord>;

//     fn into_iter(self) -> Self::IntoIter {
//         self.nodes().iter()
//     }
// }

// impl <'i, 's> IntoIterator for &'i DStar<'s> {
//     type Item = &'i Coord;
//     type IntoIter = Iter<'i, Coord>;

//     fn into_iter(self) -> Self::IntoIter {
//         self.nodes().iter().rev()
//     }
// }





// impl Iterator for Plan {
// }

fn main() {
    let x = vec![(1,1), (2,2), (3,3)];
    let astar_plan = AStarPlan { plan: x.clone() };
    let x = vec![(3,3), (2,2), (1,1)];
    let dstar_plan = DStarPlan { plan: x.clone() };
    assert!(
        astar_plan.iter().zip(dstar_plan.iter()).all(|(a, d)| a == d),
    );
}
