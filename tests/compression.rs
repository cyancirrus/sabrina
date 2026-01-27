use sabrina::environment::quad::QuadTree;
use sabrina::global::types::Belief;

#[test]
fn test_compression() {
    let levels = 2;
    let mut map = QuadTree::initialize(levels);
    map.insert_cell(&(1, 1), Belief::Occupied);
    assert_eq!(map.information.len(), 4);
    map.insert_cell(&(1, 0), Belief::Occupied);
    assert_eq!(map.information.len(), 4);
    map.insert_cell(&(0, 0), Belief::Occupied);
    assert_eq!(map.information.len(), 4);
    map.insert_cell(&(0, 1), Belief::Occupied);
    assert_eq!(map.get_cell(&(1, 1)), Some((1, Belief::Occupied)));
    assert_eq!(map.information.len(), 1);
    map.insert_cell(&(1, 1), Belief::Free);
    assert_eq!(map.information.len(), 4);
    map.insert_cell(&(1, 0), Belief::Free);
    map.insert_cell(&(0, 0), Belief::Free);
    map.insert_cell(&(0, 1), Belief::Free);
    assert_eq!(map.information.len(), 1);
    assert_eq!(map.get_cell(&(1, 1)), Some((1, Belief::Free)));
}
