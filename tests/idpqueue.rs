use sabrina::global::types::indirect_pqueue::IPQueue;
use sabrina::global::types::StarKey;

#[test]
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

#[test]
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

#[test]
fn ordering() {
    let mut pqueue:IPQueue<StarKey, usize> = IPQueue::new();
    pqueue.push(
        1,
        StarKey {
            cost_astar:0,
            cost_dijkstra: 0,
        },
    );
    pqueue.push(
        0,
        StarKey {
            cost_astar:20,
            cost_dijkstra: 20,
        },
    );
    let (identity, _) = pqueue.pop().unwrap();
    assert_eq!(1, identity)
}
