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
impl Lidar {
    pub fn new(max_range: usize, oracle: Grid) -> Self {
        Self { max_range, oracle }
    }
    pub fn measure(&self, position: Coord) -> Measurement {
        let mut data = [None; GRAIN];
        // polar order of scan ie counter-clockwise
        for (h, &d) in [(1, 0), (0, 1), (!0, 0), (0, !0)].iter().enumerate() {
            data[h] = self.oracle.raycast(position, d, self.max_range);
        }
        Measurement { data }
    }
}
