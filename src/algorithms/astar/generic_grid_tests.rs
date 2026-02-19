use super::*;

use crate::utils::heuristics::heuristic_get_range_to;
use screeps::constants::Terrain;
use screeps::local::{Position, RoomCoordinate, RoomName, RoomXY};

use crate::common::data::load_all_room_terrains_from_map;

use itertools::Itertools;

// Helper Functions

fn new_position(room_name: &str, x: u8, y: u8) -> Position {
    Position::new(
        RoomCoordinate::try_from(x).unwrap(),
        RoomCoordinate::try_from(y).unwrap(),
        room_name.parse().unwrap(),
    )
}

fn all_tiles_are_plains_costs<T>(_prev: T, _node: T) -> Option<u32> {
    Some(1)
}

fn all_tiles_are_swamps_costs<T>(_prev: T, _node: T) -> Option<u32> {
    Some(5)
}

fn is_goal_fn<T: std::cmp::PartialEq>(goal: T) -> impl Fn(T) -> bool {
    move |node: T| node == goal
}

// Testing function where all tiles are reachable except for (10, 12)
fn roomxy_unreachable_tile_costs(_prev: RoomXY, node: RoomXY) -> Option<u32> {
    if node.x.u8() == 10 && node.y.u8() == 12 {
        None
    } else {
        Some(1)
    }
}

// Testing function where all tiles are reachable except for (10, 12)
fn position_unreachable_tile_costs(_prev: Position, node: Position) -> Option<u32> {
    if node.x().u8() == 10 && node.y().u8() == 12 {
        None
    } else {
        Some(1)
    }
}

// Test Cases

#[test]
fn gridstate_orders_comparisons_by_f_score_descending() {
    // Test that f-scores always sort in descending order (greater values return
    // Ordering::Less), regardless of g-score or position
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

            let res = a.cmp(&b);
            // Remember, f_score orderings are intended to be reversed/descending, so since a.f_score < b.f_score, a > b
            assert_eq!(res, Ordering::Greater);

            let res = b.cmp(&a);
            assert_eq!(res, Ordering::Less);
        }
    }
}

#[test]
fn gridstate_orders_comparisons_by_g_score_descending_for_equal_f_scores() {
    // Test that g-score tie-breaking always sorts in ascending order (lesser values return
    // Ordering::Less), regardless of position
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

            let res = a.cmp(&b);
            assert_eq!(res, Ordering::Less);

            let res = b.cmp(&a);
            assert_eq!(res, Ordering::Greater);
        }
    }
}

#[test]
fn gridstate_orders_comparisons_by_position_ascending_for_equal_f_and_g_scores() {
    // Test that position tie-breaking always sorts in ascending order (lesser values return
    // Ordering::Less)
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

        let res = a.cmp(&b);
        assert_eq!(res, Ordering::Less);

        let res = b.cmp(&a);
        assert_eq!(res, Ordering::Greater);
    }
}

#[test]
fn simple_linear_path_roomxy() {
    let start = unsafe { RoomXY::unchecked_new(10, 10) };
    let goal = unsafe { RoomXY::unchecked_new(10, 12) };
    let search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        all_tiles_are_plains_costs,
        heuristic_get_range_to(goal),
        2000,
        2000,
        0,
    );

    assert_eq!(search_results.incomplete(), false);
    assert!(search_results.cost().is_some());
    assert_eq!(search_results.cost().unwrap(), 2);
    assert_eq!(search_results.ops() < 2000, true);

    let path = search_results.path();

    assert_eq!(path.len(), 3);

    // All three of these nodes are on a shortest path, so we
    // can't guarantee that we'll get any specific one of them
    let middle_node_1 = unsafe { RoomXY::unchecked_new(10, 11) };
    let middle_node_2 = unsafe { RoomXY::unchecked_new(11, 11) };
    let middle_node_3 = unsafe { RoomXY::unchecked_new(11, 10) };

    assert_eq!(path.contains(&start), true);
    assert_eq!(path.contains(&goal), true);

    let contains_a_middle_node = path.contains(&middle_node_1)
        | path.contains(&middle_node_2)
        | path.contains(&middle_node_3);
    assert_eq!(contains_a_middle_node, true);
}

#[test]
fn simple_linear_path_position() {
    let room_name = "E5N6";
    let start = new_position(room_name, 10, 10);
    let goal = new_position(room_name, 10, 12);
    let search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        all_tiles_are_plains_costs,
        heuristic_get_range_to(goal),
        2000,
        2000,
        0,
    );

    assert_eq!(search_results.incomplete(), false);
    assert!(search_results.cost().is_some());
    assert_eq!(search_results.cost().unwrap(), 2);
    assert_eq!(search_results.ops() < 2000, true);

    let path = search_results.path();

    assert_eq!(path.len(), 3);

    // All three of these nodes are on a shortest path, so we
    // can't guarantee that we'll get any specific one of them
    let middle_node_1 = new_position(room_name, 10, 11);
    let middle_node_2 = new_position(room_name, 11, 11);
    let middle_node_3 = new_position(room_name, 11, 10);

    assert_eq!(path.contains(&start), true);
    assert_eq!(path.contains(&goal), true);

    let contains_a_middle_node = path.contains(&middle_node_1)
        | path.contains(&middle_node_2)
        | path.contains(&middle_node_3);
    assert_eq!(contains_a_middle_node, true);
}

#[test]
fn unreachable_target_roomxy() {
    let start = unsafe { RoomXY::unchecked_new(10, 10) };
    let goal = unsafe { RoomXY::unchecked_new(10, 12) };
    let search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        roomxy_unreachable_tile_costs,
        heuristic_get_range_to(goal),
        2000,
        2000,
        0,
    );

    println!("{:?}", search_results);

    assert_eq!(search_results.incomplete(), true);
    assert!(search_results.cost().is_some());
    assert_eq!(search_results.cost().unwrap() > 0, true);
    assert_eq!(search_results.ops() == 2000, true);
}

#[test]
fn unreachable_target_position() {
    let room_name = "E5N6";
    let start = new_position(room_name, 10, 10);
    let goal = new_position(room_name, 10, 12);
    let search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        position_unreachable_tile_costs,
        heuristic_get_range_to(goal),
        2000,
        2000,
        0,
    );

    println!("{:?}", search_results);

    assert_eq!(search_results.incomplete(), true);
    assert!(search_results.cost().is_some());
    assert_eq!(search_results.cost().unwrap() > 0, true);
    assert_eq!(search_results.ops() > 0, true);
}

#[test]
fn max_ops_halt_roomxy() {
    let max_ops_failure = 5;
    let max_ops_success = 100;
    let start = unsafe { RoomXY::unchecked_new(10, 10) };
    let goal = unsafe { RoomXY::unchecked_new(30, 30) }; // This target generally takes ~20 ops to find

    // Failure case
    let search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        all_tiles_are_plains_costs,
        heuristic_get_range_to(goal),
        max_ops_failure,
        2000,
        0,
    );

    assert_eq!(search_results.incomplete(), true);
    assert!(search_results.cost().is_some());
    assert_eq!(search_results.cost().unwrap() > 0, true);
    assert_eq!(search_results.ops() == max_ops_failure, true);

    // Success case
    let search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        all_tiles_are_plains_costs,
        heuristic_get_range_to(goal),
        max_ops_success,
        2000,
        0,
    );

    assert_eq!(search_results.incomplete(), false);
    assert!(search_results.cost().is_some());
    assert_eq!(search_results.cost().unwrap() > 0, true);
    assert_eq!(search_results.ops() < max_ops_success, true);

    let path = search_results.path();

    assert_eq!(path.len(), 21);
}

#[test]
fn max_ops_halt_position() {
    let max_ops_failure = 5;
    let max_ops_success = 100;
    let room_name = "E5N6";
    let start = new_position(room_name, 10, 10);
    let goal = new_position(room_name, 30, 30); // This target generally takes ~20 ops to find

    // Failure case
    let search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        all_tiles_are_plains_costs,
        heuristic_get_range_to(goal),
        max_ops_failure,
        2000,
        0,
    );

    assert_eq!(search_results.incomplete(), true);
    assert_eq!(search_results.cost().unwrap() > 0, true);
    assert_eq!(search_results.ops() == max_ops_failure, true);

    // Success case
    let search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        all_tiles_are_plains_costs,
        heuristic_get_range_to(goal),
        max_ops_success,
        2000,
        0,
    );

    assert_eq!(search_results.incomplete(), false);
    assert!(search_results.cost().is_some());
    assert_eq!(search_results.cost().unwrap() > 0, true);
    assert_eq!(search_results.ops() < max_ops_success, true);

    let path = search_results.path();

    assert_eq!(path.len(), 21);
}

#[test]
fn max_cost_halt_roomxy() {
    let max_cost_failure = 5;
    let max_cost_success = 100;
    let start = unsafe { RoomXY::unchecked_new(10, 10) };
    let goal = unsafe { RoomXY::unchecked_new(10, 12) }; // This target will cost 10 to move to

    // Failure case
    let search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        all_tiles_are_swamps_costs,
        heuristic_get_range_to(goal),
        2000,
        max_cost_failure,
        0,
    );

    assert_eq!(search_results.incomplete(), true);
    assert_eq!(search_results.ops() < 2000, true);

    // Success case
    let search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        all_tiles_are_swamps_costs,
        heuristic_get_range_to(goal),
        2000,
        max_cost_success,
        0,
    );

    assert_eq!(search_results.incomplete(), false);
    assert!(search_results.cost().is_some());
    assert_eq!(search_results.cost().unwrap() < max_cost_success, true);
    assert_eq!(search_results.ops() < 2000, true);

    let path = search_results.path();

    assert_eq!(path.len(), 3);
}

#[test]
fn max_cost_halt_position() {
    let max_cost_failure = 5;
    let max_cost_success = 100;
    let room_name = "E5N6";
    let start = new_position(room_name, 10, 10);
    let goal = new_position(room_name, 10, 12); // This target will cost 10 to move to

    // Failure case
    let search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        all_tiles_are_swamps_costs,
        heuristic_get_range_to(goal),
        2000,
        max_cost_failure,
        0,
    );
    println!("{:?}", search_results);
    assert_eq!(search_results.incomplete(), true);
    assert_eq!(search_results.ops() < 2000, true);

    // Success case
    let search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        all_tiles_are_swamps_costs,
        heuristic_get_range_to(goal),
        2000,
        max_cost_success,
        0,
    );

    assert_eq!(search_results.incomplete(), false);
    assert!(search_results.cost().is_some());
    assert_eq!(search_results.cost().unwrap() < max_cost_success, true);
    assert_eq!(search_results.ops() < 2000, true);

    let path = search_results.path();

    assert_eq!(path.len(), 3);
}

#[test]
fn validate_path_is_optimal_for_mmo_shard3_W49N34() {
    let terrains_map = load_all_room_terrains_from_map("test-rooms.json");
    let room_name_str = "W49N34";

    let room_name = RoomName::new(room_name_str).unwrap();

    let terrain_data = terrains_map.get(&room_name).unwrap();

    let start = unsafe { RoomXY::unchecked_new(6, 0) }; // Top-left exit tile
    let goal = unsafe { RoomXY::unchecked_new(40, 49) }; // Bottom-right exit tile

    let plain_cost = 1;
    let swamp_cost = 5;
    let costs = crate::utils::movement_costs::get_movement_cost_lcm_from_terrain(
        &terrain_data,
        plain_cost,
        swamp_cost,
    );
    let costs_fn = crate::utils::movement_costs::movement_costs_from_lcm(&costs);
    let neighbors_fn = crate::utils::neighbors::room_xy_neighbors;
    let max_ops = 2000;
    let max_cost = 2000;

    let dijkstra_search_results = crate::algorithms::dijkstra::shortest_path_generic(
        &[start],
        &is_goal_fn(goal),
        &costs_fn,
        neighbors_fn,
        max_ops,
        max_cost,
    );

    assert_eq!(dijkstra_search_results.incomplete(), false);
    let dijkstra_path = dijkstra_search_results.path();

    let new_cost_fn = optionize_cost_fn_results(ignore_first_param_cost_fn(costs_fn));

    let astar_search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        new_cost_fn,
        heuristic_get_range_to(goal),
        max_ops,
        max_cost,
        0,
    );

    assert_eq!(astar_search_results.incomplete(), false);
    let astar_path = astar_search_results.path();

    assert_eq!(
        astar_search_results.cost().unwrap(),
        dijkstra_search_results.cost()
    );
}

#[test]
fn validate_path_is_optimal_for_mmo_shard3_W49N34_known_edge_case() {
    let terrains_map = load_all_room_terrains_from_map("test-rooms.json");
    let room_name_str = "W49N34";

    let room_name = RoomName::new(room_name_str).unwrap();

    let terrain_data = terrains_map.get(&room_name).unwrap();

    let start = unsafe { RoomXY::unchecked_new(6, 0) };
    let goal = unsafe { RoomXY::unchecked_new(32, 5) };

    let plain_cost = 1;
    let swamp_cost = 5;
    let costs = crate::utils::movement_costs::get_movement_cost_lcm_from_terrain(
        &terrain_data,
        plain_cost,
        swamp_cost,
    );
    let costs_fn = crate::utils::movement_costs::movement_costs_from_lcm(&costs);
    let neighbors_fn = crate::utils::neighbors::room_xy_neighbors;
    let max_ops = 2000;
    let max_cost = 2000;

    let dijkstra_search_results = crate::algorithms::dijkstra::shortest_path_generic(
        &[start],
        &is_goal_fn(goal),
        &costs_fn,
        neighbors_fn,
        max_ops,
        max_cost,
    );

    assert_eq!(dijkstra_search_results.incomplete(), false);
    let dijkstra_path = dijkstra_search_results.path();

    let new_cost_fn = optionize_cost_fn_results(ignore_first_param_cost_fn(costs_fn));

    let astar_search_results = shortest_path_generic_grid(
        &[start],
        &is_goal_fn(goal),
        new_cost_fn,
        heuristic_get_range_to(goal),
        max_ops,
        max_cost,
        0,
    );

    assert_eq!(astar_search_results.incomplete(), false);
    let astar_path = astar_search_results.path();

    assert_eq!(
        astar_search_results.cost().unwrap(),
        dijkstra_search_results.cost()
    );
}

#[test]
#[ignore]
fn validate_path_is_optimal_for_mmo_shard3_arbitrary_rooms() {
    let terrains_map = load_all_room_terrains_from_map("test-rooms.json");

    let mut skipped_because_plains = 0;
    let mut num_rooms = 0;
    let room_name_str = "W49N34";
    let room_name = RoomName::new(room_name_str).unwrap();
    let tmp = vec![terrains_map.get(&room_name).unwrap()];
    for terrain_data in tmp {
        //for terrain_data in terrains_map.values() {
        num_rooms += 1;

        let plains_tiles: Vec<RoomXY> = (0..50)
            .cartesian_product(0..50)
            .map(|(y, x)| unsafe { RoomXY::unchecked_new(x, y) })
            .filter(|pos| match terrain_data.get_xy(*pos) {
                Terrain::Plain => true,
                _ => false,
            })
            .collect();

        if plains_tiles.is_empty() {
            // No Plains terrain, skip this room
            skipped_because_plains += 1;
            continue;
        }

        let total_length = plains_tiles.len();
        let second_half_start = total_length / 2;

        let start_positions = &plains_tiles[0..second_half_start];
        let goal_positions = &plains_tiles[second_half_start..total_length];

        for (start_ref, goal_ref) in start_positions.iter().cartesian_product(goal_positions) {
            let start = *start_ref;
            let goal = *goal_ref;
            if start == goal {
                continue;
            }

            let plain_cost = 1;
            let swamp_cost = 5;
            let costs = crate::utils::movement_costs::get_movement_cost_lcm_from_terrain(
                &terrain_data,
                plain_cost,
                swamp_cost,
            );
            let costs_fn = crate::utils::movement_costs::movement_costs_from_lcm(&costs);
            let neighbors_fn = crate::utils::neighbors::room_xy_neighbors;
            let max_ops = 2000;
            let max_cost = 2000;

            let dijkstra_search_results = crate::algorithms::dijkstra::shortest_path_generic(
                &[start],
                &is_goal_fn(goal),
                &costs_fn,
                neighbors_fn,
                max_ops,
                max_cost,
            );

            assert_eq!(dijkstra_search_results.incomplete(), false);
            let dijkstra_path = dijkstra_search_results.path();

            let new_cost_fn = optionize_cost_fn_results(ignore_first_param_cost_fn(costs_fn));
            let always_one_heuristic = |_| 1;

            let astar_search_results = shortest_path_generic_grid(
                &[start],
                &is_goal_fn(goal),
                new_cost_fn,
                always_one_heuristic,
                //heuristic_get_range_to(goal),
                max_ops,
                max_cost,
                0,
            );

            assert_eq!(astar_search_results.incomplete(), false);
            let astar_path = astar_search_results.path();

            //assert_eq!(astar_path.len(), dijkstra_path.len(), "Dijkstra: {:?}\nA*: {:?}", dijkstra_path, astar_path);
            assert_eq!(
                astar_search_results.cost().unwrap(),
                dijkstra_search_results.cost()
            );
        }
    }

    // Assert we didn't skip all of the rooms
    assert!(skipped_because_plains < num_rooms);
}

