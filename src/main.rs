#![allow(unused)]
use sabrina::algo::a_star::AStarPlanner;
use sabrina::algo::best_first::BestFirstPlanner;
use sabrina::algo::d_star::DStarPlanner;
use sabrina::environment::grid::Grid;
use sabrina::global::types::StarKey;
use sabrina::intelligence::sabrina::Sabrina;
use sabrina::parser::grid::read_grid;
use sabrina::sensor::lidar::Lidar;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::hash::Hash;
use sabrina::global::types::indirect_pqueue::IPQueue;

// Target DStarPlan { plan: [(1, 2), (1, 3), (2, 3), (3, 3), (4, 3), (5, 3), (6, 3), (7, 3), (8, 3), (9, 3), (10, 3), (11, 3), (12, 3), (13, 3), (14, 3), (15, 3), (16, 3), (17, 3), (18, 3)] }

// TODO: I think perhaps, the lazypqueue needs to be indirect pqueue, where the queue priority is
// based upon the current belief in priority, but the data is in reference to the propogated belief
// or known new state, ie... minheap < cost, node_id> <=> hashmap<node_id, (g,rhs)>

/// Indirect Priority Queue
///
/// # Updates queue based on new information


fn test_push_pop() {
    let mut h: IPQueue<usize, usize> = IPQueue::new();

    h.push(5, 10);
    h.push(1, 20);
    assert_eq!(Some((5, 10)), h.pop());
    h.clear();
    assert_eq!(None, h.pop());

    h.push(2, 10);
    h.push(1, 20);
    h.push(2, 30);

    assert_eq!(Some((1, 20)), h.pop());
    assert_eq!(Some((2, 30)), h.pop());
    assert_eq!(None, h.pop());
}

fn test_peek() {
    let mut h: IPQueue<usize, usize> = IPQueue::new();

    h.push(5, 10);
    h.push(1, 20);
    assert_eq!(Some((5, 10)), h.peek());
    h.clear();
    assert_eq!(None, h.pop());

    h.push(5, 10);
    h.push(2, 30);

    assert_eq!(Some((5, 10)), h.peek());
    assert_eq!(Some((5, 10)), h.pop());
    assert_eq!(Some((2, 30)), h.peek());
    assert_eq!(Some((2, 30)), h.pop());
    assert_eq!(None, h.pop());
}

fn main() {
    test_peek();
    test_push_pop();

    // println!("------------------------------------");
    // println!("      Example navigation            ");
    // println!("------------------------------------");
    // let path = "./data/sample/test_nav0.map";
    // // let path = "./data/sample/test_nav1.map";
    // // let path = "./data/sample/test_imposs.map";
    // match read_grid(path) {
    //     Ok(oracle) => {
    //         let neighs = oracle.inspect_neighs((1, 1));

    //         let position = (1, 1);
    //         let target = (1, 5);

    //         // let position = (1, 1);
    //         // let target = (9, 3);

    //         // let position = (1, 1);
    //         // let target = (18, 3);

    //         // let position = (4, 1);
    //         // let target = (4, 3);

    //         let environment = Grid::new();
    //         let lidar = Lidar::new(6, oracle.clone());
    //         // let mut sabby = Sabrina::new(position, environment, lidar, BestFirstPlanner);
    //         // let mut sabby = Sabrina::new(position, environment, lidar, AStarPlanner);
    //         // let mut sabby = Sabrina::new(position, oracle.clone(), lidar, DStarPlanner::new());
    //         let mut sabby = Sabrina::new(position, environment, lidar, DStarPlanner::new());
    //         println!("absolute_environment\n{oracle}");
    //         // println!("-------------------------------");
    //         // println!("    Starting Navigation        ");
    //         // println!("-------------------------------");
    //         println!("Final Status {:?}", sabby.navigate(target));
    //         println!("Final map\n{}", sabby.environment);
    //     }
    //     Err(e) => {
    //         println!("Err\n{e:?}");
    //     }
    // }
}
