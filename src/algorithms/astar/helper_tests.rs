use super::*;

#[test]
fn compare_option_scores_orders_properly() {
    // Test the None, None case
    let res = compare_option_scores::<u8>(None, None);
    assert_eq!(res, Ordering::Equal);

    for i in 0..u8::MAX {
        let a = i;
        let b = i + 1;

        // Test the Some, None cases
        let res = compare_option_scores(Some(a), None);
        assert_eq!(res, Ordering::Less);
        let res = compare_option_scores(None, Some(a));
        assert_eq!(res, Ordering::Greater);

        // Test the < case
        let res = compare_option_scores(Some(a), Some(b));
        assert_eq!(res, Ordering::Less);

        // Test the > case
        let res = compare_option_scores(Some(b), Some(a));
        assert_eq!(res, Ordering::Greater);

        // Test the = case
        let res = compare_option_scores(Some(a), Some(a));
        assert_eq!(res, Ordering::Equal);
    }
}

#[test]
fn binary_heap_orders_state_by_f_score_descending() {
    // Test that f-scores always sort in descending order (greater values return
    // first), regardless of g-score or position
    let large_g_score: u8 = 100;
    let small_g_score: u8 = 5;
    let large_position: u8 = 40;
    let small_position: u8 = 4;

    let irrelevant_score_orderings = [
        (small_g_score, large_g_score, small_position, large_position),
        (small_g_score, large_g_score, small_position, large_position),
        (small_g_score, large_g_score, large_position, small_position),
        (large_g_score, small_g_score, large_position, large_position),
        (large_g_score, small_g_score, large_position, small_position),
        (large_g_score, small_g_score, large_position, large_position),
        (large_g_score, large_g_score, large_position, large_position),
        (large_g_score, large_g_score, large_position, small_position),
        (large_g_score, large_g_score, large_position, large_position),
    ];

    for i in 0..u8::MAX {
        let low_f_score = i;
        let high_f_score = i + 1;

        for (a_g_score, b_g_score, a_pos, b_pos) in irrelevant_score_orderings {
            let a = State {
                g_score: a_g_score,
                f_score: low_f_score,
                position: a_pos,
            };
            let b = State {
                g_score: b_g_score,
                f_score: high_f_score,
                position: b_pos,
            };

            let mut heap = BinaryHeap::from([a, b]);

            assert_eq!(heap.pop(), Some(a));
            assert_eq!(heap.pop(), Some(b));
        }
    }
}

#[test]
fn binary_heap_orders_state_by_g_score_descending_for_equal_f_scores() {
    // Test that g-score tie-breaking always sorts in descending order (greater values return
    // first), regardless of position
    let f_score: u8 = 50;
    let large_position: u8 = 40;
    let small_position: u8 = 4;

    let irrelevant_score_orderings = [
        (small_position, large_position),
        (large_position, small_position),
        (large_position, large_position),
    ];

    for i in 0..u8::MAX {
        let low_g_score = i;
        let high_g_score = i + 1;

        for (a_pos, b_pos) in irrelevant_score_orderings {
            let a = State {
                g_score: low_g_score,
                f_score: f_score,
                position: a_pos,
            };
            let b = State {
                g_score: high_g_score,
                f_score: f_score,
                position: b_pos,
            };

            let mut heap = BinaryHeap::from([a, b]);

            assert_eq!(heap.pop(), Some(b));
            assert_eq!(heap.pop(), Some(a));
        }
    }
}

#[test]
fn binary_heap_orders_state_by_position_ascending_for_equal_f_and_g_scores() {
    // Test that position tie-breaking always sorts in ascending order (lesser values return
    // first)
    let f_score: u8 = 50;
    let g_score: u8 = 5;

    for i in 0..u8::MAX {
        let low_pos = i;
        let high_pos = i + 1;

        let a = State {
            g_score: g_score,
            f_score: f_score,
            position: low_pos,
        };
        let b = State {
            g_score: g_score,
            f_score: f_score,
            position: high_pos,
        };

        let mut heap = BinaryHeap::from([a, b]);

        assert_eq!(heap.pop(), Some(b));
        assert_eq!(heap.pop(), Some(a));
    }
}

#[test]
fn binary_heap_orders_gridstate_by_f_score_descending() {
    // Test that f-scores always sort in descending order (greater values return
    // first), regardless of g-score or position
    let large_g_score: u8 = 100;
    let small_g_score: u8 = 5;
    let large_position: u8 = 40;
    let small_position: u8 = 4;

    let irrelevant_score_orderings = [
        (small_g_score, large_g_score, small_position, large_position),
        (small_g_score, large_g_score, small_position, large_position),
        (small_g_score, large_g_score, large_position, small_position),
        (large_g_score, small_g_score, large_position, large_position),
        (large_g_score, small_g_score, large_position, small_position),
        (large_g_score, small_g_score, large_position, large_position),
        (large_g_score, large_g_score, large_position, large_position),
        (large_g_score, large_g_score, large_position, small_position),
        (large_g_score, large_g_score, large_position, large_position),
    ];

    for i in 0..u8::MAX {
        let low_f_score = i;
        let high_f_score = i + 1;

        for (a_g_score, b_g_score, a_pos, b_pos) in irrelevant_score_orderings {
            let a = GridState {
                g_score: Some(a_g_score),
                f_score: Some(low_f_score),
                position: a_pos,
                open_direction: None,
            };
            let b = GridState {
                g_score: Some(b_g_score),
                f_score: Some(high_f_score),
                position: b_pos,
                open_direction: None,
            };

            let mut heap = BinaryHeap::from([a, b]);

            assert_eq!(heap.pop(), Some(a));
            assert_eq!(heap.pop(), Some(b));
        }
    }
}

#[test]
fn binary_heap_orders_gridstate_by_g_score_descending_for_equal_f_scores() {
    // Test that g-score tie-breaking always sorts in descending order (greater values return
    // first), regardless of position
    let f_score: u8 = 50;
    let large_position: u8 = 40;
    let small_position: u8 = 4;

    let irrelevant_score_orderings = [
        (small_position, large_position),
        (large_position, small_position),
        (large_position, large_position),
    ];

    for i in 0..u8::MAX {
        let low_g_score = i;
        let high_g_score = i + 1;

        for (a_pos, b_pos) in irrelevant_score_orderings {
            let a = GridState {
                g_score: Some(low_g_score),
                f_score: Some(f_score),
                position: a_pos,
                open_direction: None,
            };
            let b = GridState {
                g_score: Some(high_g_score),
                f_score: Some(f_score),
                position: b_pos,
                open_direction: None,
            };

            let mut heap = BinaryHeap::from([a, b]);

            assert_eq!(heap.pop(), Some(b));
            assert_eq!(heap.pop(), Some(a));
        }
    }
}

#[test]
fn binary_heap_orders_gridstate_by_position_ascending_for_equal_f_and_g_scores() {
    // Test that position tie-breaking always sorts in ascending order (lesser values return
    // first)
    let f_score: u8 = 50;
    let g_score: u8 = 5;

    for i in 0..u8::MAX {
        let low_pos = i;
        let high_pos = i + 1;

        let a = GridState {
            g_score: Some(g_score),
            f_score: Some(f_score),
            position: low_pos,
            open_direction: None,
        };
        let b = GridState {
            g_score: Some(g_score),
            f_score: Some(f_score),
            position: high_pos,
            open_direction: None,
        };

        let mut heap = BinaryHeap::from([a, b]);

        assert_eq!(heap.pop(), Some(b));
        assert_eq!(heap.pop(), Some(a));
    }
}
