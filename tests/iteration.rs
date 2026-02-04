use sabrina::global::types::ACoord;
use sabrina::global::types::plan::{BackwardIter, ForwardIter};

#[test]
fn test_plan_iteration() {
    let x = vec![
        ACoord { x: 1, y: 1 },
        ACoord { x: 2, y: 2 },
        ACoord { x: 3, y: 3 },
    ];
    let forward_plan = ForwardIter::new(&x);
    assert!(forward_plan.zip(x.iter()).all(|(a, s)| a == s),);
    let reference_plan = ForwardIter::new(&x);
    let y = vec![
        ACoord { x: 3, y: 3 },
        ACoord { x: 2, y: 2 },
        ACoord { x: 1, y: 1 },
    ];
    let backward_plan = BackwardIter::new(&y);
    assert!(reference_plan.zip(backward_plan).all(|(a, d)| a == d),);
}
