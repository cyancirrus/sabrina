use sabrina::environment::quad::QuadTree;
use sabrina::global::types::{ACoord, Belief};

#[test]
fn test_compression() {
    let levels = 2;
    let mut map = QuadTree::init(levels);
    map.update_belief(&ACoord { x: 1, y: 1 }, Belief::Occupied);
    assert_eq!(map.information.len(), 4);
    map.update_belief(&ACoord { x: 1, y: 0 }, Belief::Occupied);
    assert_eq!(map.information.len(), 4);
    map.update_belief(&ACoord { x: 0, y: 0 }, Belief::Occupied);
    assert_eq!(map.information.len(), 4);
    map.update_belief(&ACoord { x: 0, y: 1 }, Belief::Occupied);
    assert_eq!(
        map.get_coord(ACoord { x: 1, y: 1 }),
        Some((1, Belief::Occupied))
    );
    assert_eq!(map.information.len(), 1);
    map.update_belief(&ACoord { x: 1, y: 1 }, Belief::Free);
    assert_eq!(map.information.len(), 4);
    map.update_belief(&ACoord { x: 1, y: 0 }, Belief::Free);
    map.update_belief(&ACoord { x: 0, y: 0 }, Belief::Free);
    map.update_belief(&ACoord { x: 0, y: 1 }, Belief::Free);
    assert_eq!(map.information.len(), 1);
    assert_eq!(map.get_coord(ACoord { x: 1, y: 1 }), Some((1, Belief::Free)));
}
