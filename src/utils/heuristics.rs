use crate::common::traits::GetRangeTo;

/// Helper function to create a heuristic cost function closure for a single goal node.
///
/// This heuristic cost is simply the range between the provided node and the goal node.
///
/// If using the convenience functions for common pathfinding use-cases,
/// you will not normally need to use this function.
///
/// # Examples
/// ```rust
/// use screeps::RoomXY;
/// use screeps_pathfinding::utils::goals::goal_exact_node;
/// use screeps_pathfinding::utils::heuristics::heuristic_get_range_to;
///
/// let start = RoomXY::checked_new(24, 18).unwrap();
/// let goal = RoomXY::checked_new(34, 40).unwrap();
/// let cost_fn = |_| 1;
/// screeps_pathfinding::algorithms::astar::shortest_path_roomxy(
///     start,
///     &goal_exact_node(goal),
///     cost_fn,
///     &heuristic_get_range_to(goal),
/// );
pub fn heuristic_get_range_to<T: GetRangeTo + Copy + 'static>(goal: T) -> impl Fn(T) -> u32 {
    move |node: T| node.get_range_to(goal)
}

/// Helper function to create a heuristic cost function closure for multiple goal nodes.
///
/// This heuristic cost is simply the minimum range between the provided node and each of the goal nodes.
///
/// If using the convenience functions for common pathfinding use-cases,
/// you will not normally need to use this function.
///
/// # Examples
/// ```rust
/// use screeps::RoomXY;
/// use screeps_pathfinding::utils::goals::goal_exact_node_multigoal;
/// use screeps_pathfinding::utils::heuristics::heuristic_get_range_to_multigoal;
///
/// let start = RoomXY::checked_new(24, 18).unwrap();
/// let goal_a = RoomXY::checked_new(34, 40).unwrap();
/// let goal_b = RoomXY::checked_new(34, 45).unwrap();
/// let goals = &[goal_a, goal_b];
/// let cost_fn = |_| 1;
/// screeps_pathfinding::algorithms::astar::shortest_path_roomxy(
///     start,
///     &goal_exact_node_multigoal(goals),
///     cost_fn,
///     &heuristic_get_range_to_multigoal(goals),
/// );
pub fn heuristic_get_range_to_multigoal<T: GetRangeTo + Copy>(
    goals: &[T],
) -> impl Fn(T) -> u32 + '_ {
    |node: T| {
        goals
            .iter()
            .map(|g| node.get_range_to(*g))
            .min()
            .unwrap_or(0)
    }
}
