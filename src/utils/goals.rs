use crate::common::traits::GetRangeTo;

/// Helper function to create a goal function closure for exact node matching with a single goal node.
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
/// let cost_fn = |_| Some(1);
/// screeps_pathfinding::algorithms::astar::shortest_path_roomxy(
///     start,
///     &goal_exact_node(goal),
///     cost_fn,
///     &heuristic_get_range_to(goal),
/// );
pub fn goal_exact_node<T: std::cmp::PartialEq + 'static>(goal: T) -> impl Fn(T) -> bool {
    move |node: T| node == goal
}

/// Helper function to create a goal function closure for exact node matching with multiple goal nodes.
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
/// let cost_fn = |_| Some(1);
/// screeps_pathfinding::algorithms::astar::shortest_path_roomxy(
///     start,
///     &goal_exact_node_multigoal(goals),
///     cost_fn,
///     &heuristic_get_range_to_multigoal(goals),
/// );
pub fn goal_exact_node_multigoal<T: std::cmp::PartialEq>(goals: &[T]) -> impl Fn(T) -> bool + '_ {
    |node: T| goals.contains(|g| node == *g)
}

/// Helper function to create a goal function closure for matching
/// nodes <= than the specified range from a single goal node.
///
/// If using the convenience functions for common pathfinding use-cases,
/// you will not normally need to use this function.
///
/// # Examples
/// ```rust
/// use screeps::RoomXY;
/// use screeps_pathfinding::utils::goals::goal_range_to_node;
/// use screeps_pathfinding::utils::heuristics::heuristic_get_range_to;
///
/// let start = RoomXY::checked_new(24, 18).unwrap();
/// let goal = RoomXY::checked_new(34, 40).unwrap();
/// let cost_fn = |_| Some(1);
/// screeps_pathfinding::algorithms::astar::shortest_path_roomxy(
///     start,
///     &goal_range_to_node(goal, 3),
///     cost_fn,
///     &heuristic_get_range_to(goal),
/// );
pub fn goal_range_to_node<T: GetRangeTo + Copy + 'static>(
    goal: T,
    range: u32,
) -> impl Fn(T) -> bool {
    move |node: T| node.get_range_to(goal) <= range
}

/// Helper function to create a goal function closure for matching
/// nodes <= than the specified range from any of multiple goal nodes.
///
/// If using the convenience functions for common pathfinding use-cases,
/// you will not normally need to use this function.
///
/// # Examples
/// ```rust
/// use screeps::RoomXY;
/// use screeps_pathfinding::utils::goals::goal_range_to_node_multigoal;
/// use screeps_pathfinding::utils::heuristics::heuristic_get_range_to_multigoal;
///
/// let start = RoomXY::checked_new(24, 18).unwrap();
/// let goal_a = RoomXY::checked_new(34, 40).unwrap();
/// let goal_b = RoomXY::checked_new(34, 45).unwrap();
/// let goals = &[goal_a, goal_b];
/// let cost_fn = |_| Some(1);
/// screeps_pathfinding::algorithms::astar::shortest_path_roomxy(
///     start,
///     &goal_range_to_node_multigoal(goals, 3),
///     cost_fn,
///     &heuristic_get_range_to_multigoal(goals),
/// );
pub fn goal_range_to_node_multigoal<T: GetRangeTo + Copy>(
    goals: &[T],
    range: u32,
) -> impl Fn(T) -> bool + '_ {
    move |node: T| goals.iter().any(|g| node.get_range_to(*g) <= range)
}
