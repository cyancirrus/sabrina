use sabrina::global::types::plan::{ForwardIter, BackwardIter};


#[test]
fn test_plan_iteration() {
    let x = vec![(1, 1), (2, 2), (3, 3)];
    let forward_plan = ForwardIter::new(&x );
    assert!(
        forward_plan.zip(x.iter()).all(|(a, s)| a == s),

    );
    let reference_plan = ForwardIter::new(&x );
    let y = vec![(3, 3), (2, 2), (1, 1)];
    let backward_plan = BackwardIter::new(&y );
    assert!(
        reference_plan
            
            .zip(backward_plan)
            .all(|(a, d)| a == d),
    );
}
