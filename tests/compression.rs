use sabrina::environment::quad::QuadTree;
use sabrina::global::types::Belief;

#[test]
fn test_compression() {
    let levels = 2;
    let mut map = QuadTree::initialize(levels);
    map.update_belief(&(1, 1), Belief::Occupied);
    assert_eq!(map.information.len(), 4);
    map.update_belief(&(1, 0), Belief::Occupied);
    assert_eq!(map.information.len(), 4);
    map.update_belief(&(0, 0), Belief::Occupied);
    assert_eq!(map.information.len(), 4);
    map.update_belief(&(0, 1), Belief::Occupied);
    assert_eq!(map.get_cell(&(1, 1)), Some((1, Belief::Occupied)));
    assert_eq!(map.information.len(), 1);
    map.update_belief(&(1, 1), Belief::Free);
    assert_eq!(map.information.len(), 4);
    map.update_belief(&(1, 0), Belief::Free);
    map.update_belief(&(0, 0), Belief::Free);
    map.update_belief(&(0, 1), Belief::Free);
    assert_eq!(map.information.len(), 1);
    assert_eq!(map.get_cell(&(1, 1)), Some((1, Belief::Free)));
}
