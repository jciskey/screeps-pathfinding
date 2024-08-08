// https://en.wikipedia.org/wiki/A*_search_algorithm

// Sample code pulled (and modified) from: https://doc.rust-lang.org/nightly/std/collections/binary_heap/index.html#examples

use std::cmp::Ordering;
use std::collections::{
    hash_map::Entry,
    {BinaryHeap, HashMap},
};
use std::hash::Hash;

use screeps::constants::Direction;
use screeps::{Position, RoomXY};

use crate::common::traits::AddDirection;
use crate::utils::goals::goal_exact_node_multigoal;
use crate::utils::heuristics::heuristic_get_range_to_multigoal;

/// A simple trait encapsulating what other traits are needed
/// for a type to be usable in the A* Algorithm.
pub trait AStarNode: Eq + Hash + Copy + Ord + AddDirection {}
impl<T> AStarNode for T where T: Eq + Hash + Copy + Ord + AddDirection {}

#[derive(Debug)]
pub struct AStarSearchResults<T>
where
    T: AStarNode,
{
    ops_used: u32,
    cost: u32,
    incomplete: bool,
    path: Vec<T>,
}

impl<T: AStarNode> AStarSearchResults<T> {
    /// The number of expand node operations used
    pub fn ops(&self) -> u32 {
        self.ops_used
    }

    /// The movement cost of the result path
    pub fn cost(&self) -> u32 {
        self.cost
    }

    /// Whether the path contained is incomplete
    pub fn incomplete(&self) -> bool {
        self.incomplete
    }

    /// A shortest path from the start node to the goal node
    pub fn path(&self) -> &[T] {
        &self.path
    }
}

#[derive(Copy, Clone, Eq, PartialEq)]
struct State<T>
where
    T: Ord,
{
    /// cost to reach this position (the g_score in A* terminology)
    g_score: u32,
    /// f_score is the sum of the known cost to reach this position (the g_score) plus the estimated cost remaining from this position in the best possible case
    f_score: u32,
    /// track the direction this entry was opened from, so that we're able to check
    /// only optimal moves (3 or 5 positions instead of all 8)
    open_direction: Option<Direction>,
    position: T,
}

// The priority queue depends on `Ord`.
// Explicitly implement the trait so the queue becomes a min-heap
// instead of a max-heap.
impl<T> Ord for State<T>
where
    T: Ord,
{
    fn cmp(&self, other: &Self) -> Ordering {
        // Notice that we flip the ordering on costs.
        // In case of a tie we compare positions - this step is necessary
        // to make implementations of `PartialEq` and `Ord` consistent.
        other
            .f_score
            .cmp(&self.f_score)
            .then_with(|| self.position.cmp(&other.position))
    }
}

// `PartialOrd` needs to be implemented as well.
impl<T> PartialOrd for State<T>
where
    T: Ord,
{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn check_directions<T: AStarNode, G, F>(
    start: T,
    g_score: u32,
    directions: &[Direction],
    cost_fn: G,
    heuristic_fn: F,
    heap: &mut BinaryHeap<State<T>>,
    parents: &mut HashMap<T, T>,
) where
    G: Fn(T) -> u32,
    F: Fn(T) -> u32,
{
    for direction in directions.iter().copied() {
        let Some(check_pos) = start.checked_add_direction(direction) else {
            continue;
        };

        if let Entry::Vacant(v) = parents.entry(check_pos) {
            v.insert(start);
            let next_tile_cost = cost_fn(check_pos);

            let g_score = g_score.saturating_add(next_tile_cost);

            // u32::MAX is our sentinel value for unpassable (or we've saturated the above add), skip this neighbor
            if g_score == u32::MAX {
                continue;
            }

            // let f_score = g_score.saturating_add(check_pos.get_range_heuristic(goal));
            let f_score = g_score.saturating_add(heuristic_fn(check_pos));

            heap.push(State {
                g_score,
                f_score,
                position: check_pos,
                open_direction: Some(direction),
            });
        }
    }
}

/// Highly-generic implementation of A* search algorithm.
/// Allows multiple starting nodes, a generic goal function,
/// generic cost and heuristic functions, control over
/// the maximum operations performed while searching,
/// and the maximum travel cost the final path can have.
///
/// Generally, you should not need to use this directly; use
/// one of the convenience functions instead:
/// [shortest_path_roomxy_single_goal] and [shortest_path_roomxy_multiple_goals]
pub fn shortest_path_generic<T: AStarNode, P, G, F>(
    start: &[T],
    goal_fn: &P,
    cost_fn: G,
    heuristic_fn: F,
    max_ops: u32,
    max_cost: u32,
) -> AStarSearchResults<T>
where
    P: Fn(T) -> bool,
    G: Fn(T) -> u32,
    F: Fn(T) -> u32,
{
    let mut remaining_ops: u32 = max_ops;
    let mut best_reached = start[0];
    let mut best_reached_f_score = u32::MAX;

    // Build this once, instead of repeatedly inside the node expansions loop
    let all_directions_iter = Direction::iter();
    let all_directions = all_directions_iter.as_slice();

    let mut parents: HashMap<T, T> = HashMap::new();
    let mut heap = BinaryHeap::new();

    for s in start.iter().copied() {
        let initial_open_entry = State {
            g_score: 0,
            f_score: heuristic_fn(s),
            position: s,
            open_direction: None,
        };
        heap.push(initial_open_entry);
    }

    // Examine the frontier with lower cost nodes first (min-heap)
    while let Some(State {
        g_score,
        position,
        f_score,
        open_direction,
    }) = heap.pop()
    {
        // We found the goal state, return the search results
        if goal_fn(position) {
            let path_opt = get_path_from_parents(&parents, position);
            return AStarSearchResults {
                ops_used: max_ops - remaining_ops,
                cost: g_score,
                incomplete: false,
                path: path_opt.unwrap_or_else(|| Vec::new()),
            };
        }

        // if this is the most promising path yet, mark it as the point to use for rebuild
        // if we have to return incomplete
        if f_score < best_reached_f_score {
            best_reached = position;
            best_reached_f_score = f_score;
        }

        remaining_ops -= 1;

        // Stop searching if we've run out of remaining ops we're allowed to perform
        if remaining_ops == 0 {
            break;
        }

        // don't evaluate children if we're beyond the maximum cost
        if g_score >= max_cost {
            continue;
        }

        let directions: &[Direction] = if let Some(open_direction) = open_direction {
            // we know what direction this tile was opened from; only explore the tiles
            // that might potentially be optimal moves
            if open_direction.is_diagonal() {
                // diagonal move; 2 rotations away might be optimal, while other moves would always
                // be more efficiently reached without traversing this tile:
                // ↖↑↗
                // ←●
                // ↙ ↖
                &[
                    open_direction,
                    open_direction.multi_rot(1),
                    open_direction.multi_rot(-1),
                    open_direction.multi_rot(2),
                    open_direction.multi_rot(-2),
                ]
            } else {
                // orthogonal move; only continuing straight or turning 45 degrees can be optimal
                // and should be explored; 90 degree moves would be more efficient as diagonal
                // moves from the parent tile without traversing this tile:
                //   ↗
                // →●→
                //   ↘
                &[
                    open_direction,
                    open_direction.multi_rot(1),
                    open_direction.multi_rot(-1),
                ]
            }
        } else {
            // didn't start with a direction, this is probably the start tile; check all directions
            all_directions
        };

        check_directions(
            position,
            g_score,
            directions,
            &cost_fn,
            &heuristic_fn,
            &mut heap,
            &mut parents,
        );
    }

    // Goal not reachable
    let path_opt = get_path_from_parents(&parents, best_reached);
    AStarSearchResults {
        ops_used: max_ops - remaining_ops,
        cost: best_reached_f_score,
        incomplete: true,
        path: path_opt.unwrap_or_else(|| Vec::new()),
    }
}

fn get_path_from_parents<T: AStarNode>(parents: &HashMap<T, T>, end: T) -> Option<Vec<T>> {
    let mut path = Vec::new();

    let mut current_pos = end;

    path.push(end);

    let mut parent_opt = parents.get(&current_pos);
    while parent_opt.is_some() {
        let parent = parent_opt.unwrap();
        path.push(*parent);
        current_pos = *parent;
        parent_opt = parents.get(&current_pos);
    }

    Some(path.into_iter().rev().collect())
}

/// Convenience function for the common use-case of searching
/// from a single starting node to a single goal node.
///
/// Uses sane default values for maximum operations and travel costs.
/// For more fine-grained control, see: [shortest_path_generic]
///
/// # Example
/// ```rust
/// use screeps::{LocalRoomTerrain, RoomXY};
///
/// let start = RoomXY::checked_new(24, 18).unwrap();
/// let goal = RoomXY::checked_new(34, 40).unwrap();
/// let room_terrain = LocalRoomTerrain::new_from_bits(Box::new([0; 2500])); // Terrain that's all plains
/// let plain_cost = 1;
/// let swamp_cost = 5;
/// let costs = screeps_pathfinding::utils::movement_costs::get_movement_cost_lcm_from_terrain(&room_terrain, plain_cost, swamp_cost);
/// let costs_fn = screeps_pathfinding::utils::movement_costs::movement_costs_from_lcm(&costs);
///
/// let search_results = screeps_pathfinding::algorithms::astar::shortest_path_roomxy_single_goal(
///     start,
///     goal,
///     costs_fn,
/// );
///
/// if !search_results.incomplete() {
///   let path = search_results.path();
///   println!("Path: {:?}", path);
/// }
/// else {
///   println!("Could not find A* shortest path.");
///   println!("Search Results: {:?}", search_results);
/// }
/// ```
pub fn shortest_path_roomxy_single_goal<G>(
    start: RoomXY,
    goal: RoomXY,
    cost_fn: G,
) -> AStarSearchResults<RoomXY>
where
    G: Fn(RoomXY) -> u32,
{
    shortest_path_roomxy_multiple_goals(start, &[goal], cost_fn)
}

/// Convenience function for the common use-case of searching
/// from a single starting node to multiple goal nodes.
///
/// Uses sane default values for maximum operations and travel costs.
/// For more fine-grained control, see: [shortest_path_generic]
///
/// # Example
/// ```rust
/// use screeps::{LocalRoomTerrain, RoomXY};
///
/// let start = RoomXY::checked_new(24, 18).unwrap();
/// let goal_a = RoomXY::checked_new(34, 40).unwrap();
/// let goal_b = RoomXY::checked_new(34, 45).unwrap();
/// let room_terrain = LocalRoomTerrain::new_from_bits(Box::new([0; 2500])); // Terrain that's all plains
/// let plain_cost = 1;
/// let swamp_cost = 5;
/// let costs = screeps_pathfinding::utils::movement_costs::get_movement_cost_lcm_from_terrain(&room_terrain, plain_cost, swamp_cost);
/// let costs_fn = screeps_pathfinding::utils::movement_costs::movement_costs_from_lcm(&costs);
///
/// let search_results = screeps_pathfinding::algorithms::astar::shortest_path_roomxy_multiple_goals(
///     start,
///     &[goal_a, goal_b],
///     costs_fn,
/// );
///
/// if !search_results.incomplete() {
///   let path = search_results.path();
///   println!("Path: {:?}", path);
/// }
/// else {
///   println!("Could not find A* shortest path.");
///   println!("Search Results: {:?}", search_results);
/// }
/// ```
pub fn shortest_path_roomxy_multiple_goals<G>(
    start: RoomXY,
    goals: &[RoomXY],
    cost_fn: G,
) -> AStarSearchResults<RoomXY>
where
    G: Fn(RoomXY) -> u32,
{
    shortest_path_roomxy_multistart(
        &[start],
        &goal_exact_node_multigoal(goals),
        cost_fn,
        &heuristic_get_range_to_multigoal(goals),
    )
}

/// Convenience method for running single-start A* with default costs
/// while still retaining control over the heuristic function used.
pub fn shortest_path_roomxy<P, G, F>(
    start: RoomXY,
    goal_fn: &P,
    cost_fn: G,
    heuristic_fn: &F,
) -> AStarSearchResults<RoomXY>
where
    P: Fn(RoomXY) -> bool,
    G: Fn(RoomXY) -> u32,
    F: Fn(RoomXY) -> u32,
{
    shortest_path_roomxy_multistart(&[start], goal_fn, cost_fn, heuristic_fn)
}

/// Convenience method for running multi-start A* with default costs
/// while still retaining control over the heuristic function used.
pub fn shortest_path_roomxy_multistart<P, G, F>(
    start_nodes: &[RoomXY],
    goal_fn: &P,
    cost_fn: G,
    heuristic_fn: &F,
) -> AStarSearchResults<RoomXY>
where
    P: Fn(RoomXY) -> bool,
    G: Fn(RoomXY) -> u32,
    F: Fn(RoomXY) -> u32,
{
    let max_ops = 2000;
    let max_cost = 2000;
    shortest_path_generic(
        start_nodes,
        goal_fn,
        cost_fn,
        heuristic_fn,
        max_ops,
        max_cost,
    )
}

/// Convenience function for the common use-case of searching
/// from a single starting node to a single goal node.
///
/// Uses sane default values for maximum operations and travel costs.
/// For more fine-grained control, see: [shortest_path_generic]
///
/// # Example
/// ```rust
/// use screeps::{LocalRoomTerrain, Position, RoomCoordinate};
///
/// fn new_position(room_name: &str, x: u8, y: u8) -> Position {
///        Position::new(
///            RoomCoordinate::try_from(x).unwrap(),
///            RoomCoordinate::try_from(y).unwrap(),
///            room_name.parse().unwrap(),
///        )
///    }
///
/// let room_name = "E5N6";
/// let start = new_position(room_name, 24, 18);
/// let goal = new_position(room_name, 34, 40);
/// let room_terrain = LocalRoomTerrain::new_from_bits(Box::new([0; 2500])); // Terrain that's all plains
/// let plain_cost = 1;
/// let swamp_cost = 5;
/// let costs = screeps_pathfinding::utils::movement_costs::get_movement_cost_lcm_from_terrain(&room_terrain, plain_cost, swamp_cost);
/// let costs_fn = screeps_pathfinding::utils::movement_costs::movement_costs_from_lcm(&costs);
///
/// let search_results = screeps_pathfinding::algorithms::astar::shortest_path_position_single_goal(
///     start,
///     goal,
///     costs_fn,
/// );
///
/// if !search_results.incomplete() {
///   let path = search_results.path();
///   println!("Path: {:?}", path);
/// }
/// else {
///   println!("Could not find A* shortest path.");
///   println!("Search Results: {:?}", search_results);
/// }
/// ```
pub fn shortest_path_position_single_goal<G>(
    start: Position,
    goal: Position,
    cost_fn: G,
) -> AStarSearchResults<Position>
where
    G: Fn(Position) -> u32,
{
    shortest_path_position_multiple_goals(start, &[goal], cost_fn)
}

/// Convenience function for the common use-case of searching
/// from a single starting node to multiple goal nodes.
///
/// Uses sane default values for maximum operations and travel costs.
/// For more fine-grained control, see: [shortest_path_generic]
///
/// # Example
/// ```rust
/// use screeps::{LocalRoomTerrain, Position, RoomCoordinate};
///
/// fn new_position(room_name: &str, x: u8, y: u8) -> Position {
///        Position::new(
///            RoomCoordinate::try_from(x).unwrap(),
///            RoomCoordinate::try_from(y).unwrap(),
///            room_name.parse().unwrap(),
///        )
///    }
///
/// let room_name = "E5N6";
/// let start = new_position(room_name, 24, 18);
/// let goal_a = new_position(room_name, 34, 40);
/// let goal_b = new_position(room_name, 34, 45);
/// let room_terrain = LocalRoomTerrain::new_from_bits(Box::new([0; 2500])); // Terrain that's all plains
/// let plain_cost = 1;
/// let swamp_cost = 5;
/// let costs = screeps_pathfinding::utils::movement_costs::get_movement_cost_lcm_from_terrain(&room_terrain, plain_cost, swamp_cost);
/// let costs_fn = screeps_pathfinding::utils::movement_costs::movement_costs_from_lcm(&costs);
///
/// let search_results = screeps_pathfinding::algorithms::astar::shortest_path_position_multiple_goals(
///     start,
///     &[goal_a, goal_b],
///     costs_fn,
/// );
///
/// if !search_results.incomplete() {
///   let path = search_results.path();
///   println!("Path: {:?}", path);
/// }
/// else {
///   println!("Could not find A* shortest path.");
///   println!("Search Results: {:?}", search_results);
/// }
/// ```
pub fn shortest_path_position_multiple_goals<G>(
    start: Position,
    goals: &[Position],
    cost_fn: G,
) -> AStarSearchResults<Position>
where
    G: Fn(Position) -> u32,
{
    shortest_path_position_multistart(
        &[start],
        &goal_exact_node_multigoal(goals),
        cost_fn,
        &heuristic_get_range_to_multigoal(goals),
    )
}

/// Convenience method for running single-start A* with default costs
/// while still retaining control over the heuristic function used.
pub fn shortest_path_position<P, G, F>(
    start: Position,
    goal_fn: &P,
    cost_fn: G,
    heuristic_fn: &F,
) -> AStarSearchResults<Position>
where
    P: Fn(Position) -> bool,
    G: Fn(Position) -> u32,
    F: Fn(Position) -> u32,
{
    shortest_path_position_multistart(&[start], goal_fn, cost_fn, heuristic_fn)
}

/// Convenience method for running multi-start A* with default costs
/// while still retaining control over the heuristic function used.
pub fn shortest_path_position_multistart<P, G, F>(
    start_nodes: &[Position],
    goal_fn: &P,
    cost_fn: G,
    heuristic_fn: &F,
) -> AStarSearchResults<Position>
where
    P: Fn(Position) -> bool,
    G: Fn(Position) -> u32,
    F: Fn(Position) -> u32,
{
    let max_ops = 2000;
    let max_cost = 2000;
    shortest_path_generic(
        start_nodes,
        goal_fn,
        cost_fn,
        heuristic_fn,
        max_ops,
        max_cost,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::utils::heuristics::heuristic_get_range_to;
    use screeps::local::{Position, RoomCoordinate, RoomXY};

    // Helper Functions

    fn new_position(room_name: &str, x: u8, y: u8) -> Position {
        Position::new(
            RoomCoordinate::try_from(x).unwrap(),
            RoomCoordinate::try_from(y).unwrap(),
            room_name.parse().unwrap(),
        )
    }

    fn all_tiles_are_plains_costs<T>(_node: T) -> u32 {
        1
    }

    fn all_tiles_are_swamps_costs<T>(_node: T) -> u32 {
        5
    }

    fn is_goal_fn<T: std::cmp::PartialEq>(goal: T) -> impl Fn(T) -> bool {
        move |node: T| node == goal
    }

    // Testing function where all tiles are reachable except for (10, 12)
    fn roomxy_unreachable_tile_costs(node: RoomXY) -> u32 {
        if node.x.u8() == 10 && node.y.u8() == 12 {
            u32::MAX
        } else {
            1
        }
    }

    // Testing function where all tiles are reachable except for (10, 12)
    fn position_unreachable_tile_costs(node: Position) -> u32 {
        if node.x().u8() == 10 && node.y().u8() == 12 {
            u32::MAX
        } else {
            1
        }
    }

    // Test Cases

    #[test]
    fn simple_linear_path_roomxy() {
        let start = unsafe { RoomXY::unchecked_new(10, 10) };
        let goal = unsafe { RoomXY::unchecked_new(10, 12) };
        let search_results = shortest_path_generic(
            &[start],
            &is_goal_fn(goal),
            all_tiles_are_plains_costs,
            heuristic_get_range_to(goal),
            2000,
            2000,
        );

        assert_eq!(search_results.incomplete(), false);
        assert_eq!(search_results.cost(), 2);
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
        let search_results = shortest_path_generic(
            &[start],
            &is_goal_fn(goal),
            all_tiles_are_plains_costs,
            heuristic_get_range_to(goal),
            2000,
            2000,
        );

        assert_eq!(search_results.incomplete(), false);
        assert_eq!(search_results.cost(), 2);
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
        let search_results = shortest_path_generic(
            &[start],
            &is_goal_fn(goal),
            roomxy_unreachable_tile_costs,
            heuristic_get_range_to(goal),
            2000,
            2000,
        );

        println!("{:?}", search_results);

        assert_eq!(search_results.incomplete(), true);
        assert_eq!(search_results.cost() > 0, true);
        assert_eq!(search_results.ops() == 2000, true);
    }

    #[test]
    fn unreachable_target_position() {
        let room_name = "E5N6";
        let start = new_position(room_name, 10, 10);
        let goal = new_position(room_name, 10, 12);
        let search_results = shortest_path_generic(
            &[start],
            &is_goal_fn(goal),
            position_unreachable_tile_costs,
            heuristic_get_range_to(goal),
            2000,
            2000,
        );

        println!("{:?}", search_results);

        assert_eq!(search_results.incomplete(), true);
        assert_eq!(search_results.cost() > 0, true);
        assert_eq!(search_results.ops() > 0, true);
    }

    #[test]
    fn max_ops_halt_roomxy() {
        let max_ops_failure = 5;
        let max_ops_success = 100;
        let start = unsafe { RoomXY::unchecked_new(10, 10) };
        let goal = unsafe { RoomXY::unchecked_new(30, 30) }; // This target generally takes ~20 ops to find

        // Failure case
        let search_results = shortest_path_generic(
            &[start],
            &is_goal_fn(goal),
            all_tiles_are_plains_costs,
            heuristic_get_range_to(goal),
            max_ops_failure,
            2000,
        );

        assert_eq!(search_results.incomplete(), true);
        assert_eq!(search_results.cost() > 0, true);
        assert_eq!(search_results.ops() == max_ops_failure, true);

        // Success case
        let search_results = shortest_path_generic(
            &[start],
            &is_goal_fn(goal),
            all_tiles_are_plains_costs,
            heuristic_get_range_to(goal),
            max_ops_success,
            2000,
        );

        assert_eq!(search_results.incomplete(), false);
        assert_eq!(search_results.cost() > 0, true);
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
        let search_results = shortest_path_generic(
            &[start],
            &is_goal_fn(goal),
            all_tiles_are_plains_costs,
            heuristic_get_range_to(goal),
            max_ops_failure,
            2000,
        );

        assert_eq!(search_results.incomplete(), true);
        assert_eq!(search_results.cost() > 0, true);
        assert_eq!(search_results.ops() == max_ops_failure, true);

        // Success case
        let search_results = shortest_path_generic(
            &[start],
            &is_goal_fn(goal),
            all_tiles_are_plains_costs,
            heuristic_get_range_to(goal),
            max_ops_success,
            2000,
        );

        assert_eq!(search_results.incomplete(), false);
        assert_eq!(search_results.cost() > 0, true);
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
        let search_results = shortest_path_generic(
            &[start],
            &is_goal_fn(goal),
            all_tiles_are_swamps_costs,
            heuristic_get_range_to(goal),
            2000,
            max_cost_failure,
        );

        assert_eq!(search_results.incomplete(), true);
        assert_eq!(search_results.ops() < 2000, true);

        // Success case
        let search_results = shortest_path_generic(
            &[start],
            &is_goal_fn(goal),
            all_tiles_are_swamps_costs,
            heuristic_get_range_to(goal),
            2000,
            max_cost_success,
        );

        assert_eq!(search_results.incomplete(), false);
        assert_eq!(search_results.cost() < max_cost_success, true);
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
        let search_results = shortest_path_generic(
            &[start],
            &is_goal_fn(goal),
            all_tiles_are_swamps_costs,
            heuristic_get_range_to(goal),
            2000,
            max_cost_failure,
        );
        println!("{:?}", search_results);
        assert_eq!(search_results.incomplete(), true);
        assert_eq!(search_results.ops() < 2000, true);

        // Success case
        let search_results = shortest_path_generic(
            &[start],
            &is_goal_fn(goal),
            all_tiles_are_swamps_costs,
            heuristic_get_range_to(goal),
            2000,
            max_cost_success,
        );

        assert_eq!(search_results.incomplete(), false);
        assert_eq!(search_results.cost() < max_cost_success, true);
        assert_eq!(search_results.ops() < 2000, true);

        let path = search_results.path();

        assert_eq!(path.len(), 3);
    }
}
