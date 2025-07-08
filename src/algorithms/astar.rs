// https://en.wikipedia.org/wiki/A*_search_algorithm

// Sample code pulled (and modified) from: https://doc.rust-lang.org/nightly/std/collections/binary_heap/index.html#examples

use std::cmp::Ordering;
use std::collections::{
    hash_map::Entry,
    {BinaryHeap, HashMap},
};
use std::hash::Hash;

use num::CheckedAdd;

use screeps::constants::Direction;
use screeps::{Position, RoomXY};

use crate::common::traits::AddDirection;
use crate::utils::goals::goal_exact_node_multigoal;
use crate::utils::heuristics::heuristic_get_range_to_multigoal;

/// A simple trait encapsulating what other traits are needed
/// for a type to be usable in the A* Algorithm.
pub trait AStarNode: Eq + Hash + Copy + Ord {}
impl<T> AStarNode for T where T: Eq + Hash + Copy + Ord {}

/// Extends the AStarNode trait to also include the requirement for adding a direction, which is
/// sufficient to define the 8-connected grid that we use in Screeps.
pub trait AStarGridNode: AStarNode + AddDirection {}
impl<T> AStarGridNode for T where T: AStarNode + AddDirection {}

/// Helper trait for defining how we use generic costs in A*, allowing users to bring their own
/// costs without being locked into e.g. u32.
pub trait AStarCost:
    std::ops::Add<Self, Output = Self> +
    CheckedAdd +
    Copy +
    Eq +
    Sized +
    std::cmp::Ord {}
impl<T> AStarCost for T where T:
    std::ops::Add<Self, Output = Self> +
    CheckedAdd +
    Copy +
    Eq +
    Sized +
    std::cmp::Ord {}

#[derive(Debug)]
pub struct AStarSearchResults<T, O>
where
    T: AStarNode,
    O: AStarCost,
{
    ops_used: u32,
    cost: Option<O>,
    incomplete: bool,
    path: Vec<T>,
}

impl<T: AStarNode, O: AStarCost> AStarSearchResults<T, O> {
    /// The number of expand node operations used
    pub fn ops(&self) -> u32 {
        self.ops_used
    }

    /// The movement cost of the result path
    pub fn cost(&self) -> Option<O> {
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
struct State<T, O>
where
    T: Ord,
    O: AStarCost,
{
    /// cost to reach this position (the g_score in A* terminology)
    g_score: Option<O>,
    /// f_score is the sum of the known cost to reach this position (the g_score) plus the estimated cost remaining from this position in the best possible case
    f_score: Option<O>,
    /// the actual position
    position: T,
}

// The priority queue depends on `Ord`.
// Explicitly implement the trait so the queue becomes a min-heap
// instead of a max-heap.
impl<T, O> Ord for State<T, O>
where
    T: Ord,
    O: AStarCost,
{
    fn cmp(&self, other: &Self) -> Ordering {
        // Notice that we flip the ordering on costs.
        // In case of a tie we compare positions - this step is necessary
        // to make implementations of `PartialEq` and `Ord` consistent.
        match (other.f_score, self.f_score) {
            (None, None) => self.position.cmp(&other.position),
            (Some(_), None) => Ordering::Less,
            (None, Some(_)) => Ordering::Greater,
            (Some(a), Some(b)) => a.cmp(&b).then_with(|| self.position.cmp(&other.position))
        }
    }
}

// `PartialOrd` needs to be implemented as well.
impl<T, O> PartialOrd for State<T, O>
where
    T: Ord,
    O: AStarCost,
{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Copy, Clone, Eq, PartialEq)]
struct GridState<T, O>
where
    T: Ord,
    O: AStarCost,
{
    /// cost to reach this position (the g_score in A* terminology)
    g_score: Option<O>,
    /// f_score is the sum of the known cost to reach this position (the g_score) plus the estimated cost remaining from this position in the best possible case
    f_score: Option<O>,
    /// track the direction this entry was opened from, so that we're able to check
    /// only optimal moves (3 or 5 positions instead of all 8)
    open_direction: Option<Direction>,
    position: T,
}

// The priority queue depends on `Ord`.
// Explicitly implement the trait so the queue becomes a min-heap
// instead of a max-heap.
impl<T, O> Ord for GridState<T, O>
where
    T: Ord,
    O: AStarCost,
{
    fn cmp(&self, other: &Self) -> Ordering {
        // Notice that we flip the ordering on costs.
        // In case of a tie we compare positions - this step is necessary
        // to make implementations of `PartialEq` and `Ord` consistent.
        match (other.f_score, self.f_score) {
            (None, None) => self.position.cmp(&other.position),
            (Some(_), None) => Ordering::Less,
            (None, Some(_)) => Ordering::Greater,
            (Some(a), Some(b)) => a.cmp(&b).then_with(|| self.position.cmp(&other.position))
        }
    }
}

// `PartialOrd` needs to be implemented as well.
impl<T, O> PartialOrd for GridState<T, O>
where
    T: Ord,
    O: AStarCost,
{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Handles node expansion into the open set.
/// 
/// `start` is the node that is currently being expanded
/// `g_score` is the cost to get to the `start` node so far during our search
/// `neighbors` is the new nodes being considered for addition to the open set
/// `cost_fn` is a function that takes a pair of nodes and returns the cost to move from the first node to the second node, or None if the transition is invalid
/// `heuristic_fn` is a function that takes a node and returns the estimated cost to move from that node to the final goal node
/// `heap` this is the open set of nodes that are candidates for exploration/expansion
/// `parents` this is a lookup table from nodes to their parents
#[allow(clippy::too_many_arguments)]
fn expand_neighbors<T: AStarNode, G, F, O>(
    start: T,
    g_score: O,
    neighbors: &[T],
    cost_fn: G,
    heuristic_fn: F,
    heap: &mut BinaryHeap<State<T, O>>,
    parents: &mut HashMap<T, T>,
) where
    G: Fn(T, T) -> Option<O>,
    F: Fn(T) -> O,
    O: AStarCost,
{
    for neighbor in neighbors {
        if let Entry::Vacant(v) = parents.entry(*neighbor) {
            v.insert(start);
            if let Some(next_tile_cost) = cost_fn(start, *neighbor) {
                if let Some(next_g_score) = g_score.checked_add(&next_tile_cost) {
                    let raw_h_score = heuristic_fn(*neighbor);
                    let h_score = raw_h_score;
                    if let Some(f_score) = next_g_score.checked_add(&h_score) {
                        heap.push(State {
                            g_score: Some(next_g_score),
                            f_score: Some(f_score),
                            position: *neighbor,
                        });
                    } else {
                        // We've saturated the cost add, skip this neighbor
                        continue;
                    }
                } else {
                    // We've saturated the cost add, skip this neighbor
                    continue;
                }
            } else {
                // Invalid neighbor, skip
                continue;
            }
        }
    }
}

/// Handles node expansion into the open set.
/// 
/// `start` is the node that is currently being expanded
/// `g_score` is the cost to get to the `start` node so far during our search
/// `neighbors` is the new nodes being considered for addition to the open set, paired with the direction of movement from `start` to the neighbor
/// `cost_fn` is a function that takes a pair of nodes and returns the cost to move from the first node to the second node, or None if the transition is invalid
/// `heuristic_fn` is a function that takes a node and returns the estimated cost to move from that node to the final goal node
/// `heap` this is the open set of nodes that are candidates for exploration/expansion
/// `parents` this is a lookup table from nodes to their parents
#[allow(clippy::too_many_arguments)]
fn expand_grid_neighbors<T: AStarGridNode, G, F, O>(
    start: T,
    g_score: O,
    neighbors: &[(T, Direction)],
    cost_fn: G,
    heuristic_fn: F,
    heap: &mut BinaryHeap<GridState<T, O>>,
    parents: &mut HashMap<T, T>,
) where
    G: Fn(T, T) -> Option<O>,
    F: Fn(T) -> O,
    O: AStarCost,
{
    for (neighbor, direction) in neighbors {
        if let Entry::Vacant(v) = parents.entry(*neighbor) {
            v.insert(start);
            if let Some(next_tile_cost) = cost_fn(start, *neighbor) {
                if let Some(next_g_score) = g_score.checked_add(&next_tile_cost) {
                    let raw_h_score = heuristic_fn(*neighbor);
                    let h_score = raw_h_score;
                    if let Some(f_score) = next_g_score.checked_add(&h_score) {
                        heap.push(GridState {
                            g_score: Some(next_g_score),
                            f_score: Some(f_score),
                            position: *neighbor,
                            open_direction: Some(*direction),
                        });
                    } else {
                        // We've saturated the cost add, skip this neighbor
                        continue;
                    }
                } else {
                    // We've saturated the cost add, skip this neighbor
                    continue;
                }
            } else {
                // Invalid neighbor, skip
                continue;
            }
        }
    }
}

fn generate_neighbors_direction_flatten_heuristic<T, H, O>(start: T, raw_node_heuristic: H) -> impl Fn(T) -> O
where
  T: AStarGridNode,
  H: Fn(T) -> O,
  O: AStarCost + std::ops::Sub<u32, Output = O>,
{
    let all_directions_iter = Direction::iter();
    let diagonal_neighbors: Vec<T> = all_directions_iter
        .filter(|d| d.is_diagonal())
        .flat_map(|d| start.checked_add_direction(*d))
        .collect();

    move |node: T| {
        // We always need to calculate the f_score for a node
        let raw_f_score = raw_node_heuristic(node);
        match diagonal_neighbors.iter().any(|n| *n == node) {
            true => raw_f_score - 1, // If we find the node is a diagonal move, return the f_score slightly downgraded
            false => raw_f_score, // The node isn't a diagonal move, so return the f_score unchanged
        }
    }
}

/// Highly-generic implementation of A* search algorithm on a grid.
///
/// Allows multiple starting nodes, a generic goal function,
/// generic cost and heuristic functions, control over
/// the maximum operations performed while searching,
/// and the maximum travel cost the final path can have.
///
/// Generally, you should not need to use this directly; use
/// one of the convenience functions instead:
/// [shortest_path_roomxy_single_goal] and [shortest_path_roomxy_multiple_goals]
///
/// Parameters:
///
/// - `start`: A slice of the starting nodes for the search
/// - `goal_fn`: A predicate function that takes a node and returns true if the node is a goal node
/// - `cost_fn`: A function that takes in the current node and a neighbor node and returns the cost of moving to that neighbor node, or None if the transition is invalid
/// - `heuristic_fn`: A function that takes a node and returns the estimated cost of moving to the goal node from that node
/// - `max_ops`: The maximum number of expand operations to perform before stopping search
/// - `max_cost`: The maximum cost to allow for the final path before stopping search
/// - `initial_cost`: The initial cost to start the search with
pub fn shortest_path_generic_grid<T: AStarGridNode, P, G, F, O>(
    start: &[T],
    goal_fn: &P,
    cost_fn: G,
    heuristic_fn: F,
    max_ops: u32,
    max_cost: O,
    initial_cost: O,
) -> AStarSearchResults<T, O>
where
    P: Fn(T) -> bool,
    G: Fn(T, T) -> Option<O>,
    F: Fn(T) -> O,
    O: AStarCost,
{
    // let all_directions_iter = Direction::iter();
    // let diagonal_neighbors: Vec<T> = all_directions_iter
    //     .filter(|d| d.is_diagonal())
    //     .flat_map(|d| start.checked_add_direction(*d))
    //     .collect();

    let mut remaining_ops: u32 = max_ops;
    let mut best_reached = start[0];
    let mut best_reached_f_score: Option<O> = None;

    // Build this once, instead of repeatedly inside the node expansions loop
    let all_directions_iter = Direction::iter();
    let all_directions = all_directions_iter.as_slice();

    let mut parents: HashMap<T, T> = HashMap::new();
    let mut heap = BinaryHeap::new();

    for s in start.iter().copied() {
        let initial_open_entry = GridState {
            g_score: Some(initial_cost),
            f_score: Some(heuristic_fn(s)),
            position: s,
            open_direction: None,
        };
        heap.push(initial_open_entry);
    }

    // Examine the frontier with lower cost nodes first (min-heap)
    while let Some(GridState {
        g_score: g_score_opt,
        position,
        f_score: f_score_opt,
        open_direction,
    }) = heap.pop()
    {
        // We found the goal state, return the search results
        if goal_fn(position) {
            let path_opt = get_path_from_parents(&parents, position);
            return AStarSearchResults {
                ops_used: max_ops - remaining_ops,
                cost: g_score_opt,
                incomplete: false,
                path: path_opt.unwrap_or_else(|| Vec::new()),
            };
        }

        if g_score_opt.is_none() {
            // We've saturated the cost, skip this node
            continue;
        }

        let g_score = g_score_opt.unwrap();

        // Don't evaluate children if we're beyond the maximum cost
        if g_score >= max_cost {
            continue;
        }

        if f_score_opt.is_none() {
            // We've saturated the heuristic cost, skip this node
            continue;
        }

        let f_score = f_score_opt.unwrap();

        // if this is the most promising path yet, mark it as the point to use for rebuild
        // if we have to return incomplete
        // Safety: we can unwrap here because we already know it's not None
        if best_reached_f_score.map_or(true, |brfs| f_score < brfs) {
            best_reached = position;
            best_reached_f_score = Some(f_score);
        }

        remaining_ops -= 1;

        // Stop searching if we've run out of remaining ops we're allowed to perform
        if remaining_ops == 0 {
            break;
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

        let neighbors: Vec<(T, Direction)> = directions.iter()
            .map(|d| (position.checked_add_direction(*d), *d))
            .filter(|(opt, _)| opt.is_some())
            .map(|(opt, d)| (opt.unwrap(), d))
            .collect();

        expand_grid_neighbors(
            position,
            g_score,
            &neighbors,
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
/// For more fine-grained control, see: [shortest_path_generic_grid]
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
/// let costs_fn = screeps_pathfinding::utils::movement_costs::astar_movement_costs_from_lcm(&costs);
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
) -> AStarSearchResults<RoomXY, u32>
where
    G: Fn(RoomXY) -> Option<u32>,
{
    shortest_path_roomxy_multiple_goals(start, &[goal], cost_fn)
}

/// Convenience function for the common use-case of searching
/// from a single starting node to multiple goal nodes.
///
/// Uses sane default values for maximum operations and travel costs.
/// For more fine-grained control, see: [shortest_path_generic_grid]
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
/// let costs_fn = screeps_pathfinding::utils::movement_costs::astar_movement_costs_from_lcm(&costs);
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
) -> AStarSearchResults<RoomXY, u32>
where
    G: Fn(RoomXY) -> Option<u32>,
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
) -> AStarSearchResults<RoomXY, u32>
where
    P: Fn(RoomXY) -> bool,
    G: Fn(RoomXY) -> Option<u32>,
    F: Fn(RoomXY) -> u32,
{
    shortest_path_roomxy_multistart(
        &[start],
        goal_fn,
        cost_fn,
        heuristic_fn,
    )
}

/// Convenience method for running multi-start A* with default costs
/// while still retaining control over the heuristic function used.
pub fn shortest_path_roomxy_multistart<P, G, F>(
    start_nodes: &[RoomXY],
    goal_fn: &P,
    cost_fn: G,
    heuristic_fn: &F,
) -> AStarSearchResults<RoomXY, u32>
where
    P: Fn(RoomXY) -> bool,
    G: Fn(RoomXY) -> Option<u32>,
    F: Fn(RoomXY) -> u32,
{
    let max_ops = 2000;
    let max_cost = 2000;
    let new_cost_fn = ignore_first_param_cost_fn(cost_fn);
    shortest_path_generic_grid(
        start_nodes,
        goal_fn,
        new_cost_fn,
        heuristic_fn,
        max_ops,
        max_cost,
        0,
    )
}

/// Convenience function for the common use-case of searching
/// from a single starting node to a single goal node.
///
/// Uses sane default values for maximum operations and travel costs.
/// For more fine-grained control, see: [shortest_path_generic_grid]
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
/// let costs_fn = screeps_pathfinding::utils::movement_costs::astar_movement_costs_from_lcm(&costs);
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
) -> AStarSearchResults<Position, u32>
where
    G: Fn(Position) -> Option<u32>,
{
    shortest_path_position_multiple_goals(start, &[goal], cost_fn)
}

/// Convenience function for the common use-case of searching
/// from a single starting node to multiple goal nodes.
///
/// Uses sane default values for maximum operations and travel costs.
/// For more fine-grained control, see: [shortest_path_generic_grid]
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
/// let costs_fn = screeps_pathfinding::utils::movement_costs::astar_movement_costs_from_lcm(&costs);
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
) -> AStarSearchResults<Position, u32>
where
    G: Fn(Position) -> Option<u32>,
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
) -> AStarSearchResults<Position, u32>
where
    P: Fn(Position) -> bool,
    G: Fn(Position) -> Option<u32>,
    F: Fn(Position) -> u32,
{
    shortest_path_position_multistart(
        &[start],
        goal_fn,
        cost_fn,
        heuristic_fn,
    )
}

/// Convenience method for running multi-start A* with default costs
/// while still retaining control over the heuristic function used.
pub fn shortest_path_position_multistart<P, G, F>(
    start_nodes: &[Position],
    goal_fn: &P,
    cost_fn: G,
    heuristic_fn: &F,
) -> AStarSearchResults<Position, u32>
where
    P: Fn(Position) -> bool,
    G: Fn(Position) -> Option<u32>,
    F: Fn(Position) -> u32,
{
    let max_ops = 2000;
    let max_cost = 2000;

    let new_cost_fn = ignore_first_param_cost_fn(cost_fn);
    shortest_path_generic_grid(
        start_nodes,
        goal_fn,
        new_cost_fn,
        heuristic_fn,
        max_ops,
        max_cost,
        0,
    )
}

fn ignore_first_param_cost_fn<G, T, O>(cost_fn: G) -> impl Fn(T, T) -> O
where
    G: Fn(T) -> O,
{
    move |_, p| cost_fn(p)
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
}
