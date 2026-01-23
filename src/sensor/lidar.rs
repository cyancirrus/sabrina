use crate::environment::grid::Grid;
use crate::global::types::Coord;

//TODO: Next session: orientation + beam rotation OR frontier-based exploration
//TODO: When finally make quadtree, create a hazard like cost a hazard will be used as a cost
//gradient. essentially like when detect object propogate like danger upward, this way the planning
//algorithm will prefer a more racing line and will be easier for sabrina to navigate effectively,
//a*:: f(n) = g(n) + h(n); influence g(n) ie the heap priority nodes
// can also add a smoothness constraint into a* to reflect d theta^2/dt to encourage robot
// smoothness

const GRAIN: usize = 4;
// Sees in 4 principle components
pub struct Lidar {
    // Max range ould be noise informed
    pub max_range: usize,
    oracle: Grid,
}
pub struct Measurement {
    // closest objects eventually need to refactor with theta
    pub data: [Option<Coord>; GRAIN],
}

#[derive(Eq, PartialEq, Debug)]
pub enum Status {
    Enroute,
    Blocked,
    Impossible,
    Complete,
}

impl Lidar {
    pub fn new(max_range: usize, oracle: Grid) -> Self {
        Self { max_range, oracle }
    }
    fn beam(&self, position: Coord, delta: Coord) -> Option<Coord> {
        // mock interface owning interface don't need dynamic changing env at the moment
        // RcRefcell or ArcMutex if doing pathing with multiple as extensions
        let mut n_xy = position;
        for _ in 1..self.max_range {
            n_xy.0 = n_xy.0.wrapping_add(delta.0);
            n_xy.1 = n_xy.1.wrapping_add(delta.1);
            // needs to fit wrt the underlying grid
                if !self.oracle.path_clear(n_xy) {
                    // denomralize b/c is oracle and needs to be relative
                    let denorm_xy = (
                        n_xy.0.wrapping_sub(position.0),
                        n_xy.1.wrapping_sub(position.1),
                    );
                    return Some(denorm_xy);
                }
        }
        println!("--------");
        None
    }
    pub fn measure(&self, position: Coord) -> Measurement {
        let mut data = [None; GRAIN];
        // polar order of scan ie counter-clockwise
        for (h, &d) in [(1, 0), (0, 1), (!0, 0), (0, !0)].iter().enumerate() {
            data[h] = self.beam(position, d);
        }
        Measurement { data }
    }
}
