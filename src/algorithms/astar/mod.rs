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
pub trait AStarGridNode: AStarNode + AddDirection + std::fmt::Debug {}
impl<T> AStarGridNode for T where T: AStarNode + AddDirection + std::fmt::Debug {}

/// Helper trait for defining how we use generic costs in A*, allowing users to bring their own
/// costs without being locked into e.g. u32.
pub trait AStarCost:
    std::ops::Add<Self, Output = Self>
    + CheckedAdd
    + Copy
    + Eq
    + Sized
    + std::cmp::Ord
    + std::fmt::Debug
{
}
impl<T> AStarCost for T where
    T: std::ops::Add<Self, Output = Self>
        + CheckedAdd
        + Copy
        + Eq
        + Sized
        + std::cmp::Ord
        + std::fmt::Debug
{
}

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

/// A helper function to compare costs that are wrapped as Options.
///
/// If both are None, they're considered equal.
/// If one is Some, that is considered the smaller value (None == Infinity).
/// If both are Some, compare them directly.
fn compare_option_scores<O: AStarCost>(a: Option<O>, b: Option<O>) -> Ordering {
    match (a, b) {
        (None, None) => Ordering::Equal,
        (Some(_), None) => Ordering::Less,
        (None, Some(_)) => Ordering::Greater,
        (Some(l), Some(r)) => l.cmp(&r),
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
struct State<T, O>
where
    T: Ord,
    O: AStarCost,
{
    /// cost to reach this position (the g_score in A* terminology)
    g_score: O,
    /// f_score is the sum of the known cost to reach this position (the g_score) plus the estimated cost remaining from this position in the best possible case
    f_score: O,
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
        // Notice that we flip the ordering on costs for f-scores, to convert the queue into a min-heap
        // instead of a max-heap.
        other.f_score.cmp(&self.f_score).then_with(|| {
            // If the f-scores are equal, then we should tie-break with their g-scores;
            // this doesn't affect optimality, but it's a well-known optimization in the
            // literature to prefer the node with the higher g-score, since that one is almost
            // certainly closer to the target (same f && higher g -> lower h).
            self.g_score.cmp(&other.g_score).then_with(|| {
                // If the f and g scores are all equal, then we compare the positions of the nodes;
                // this gives us a guaranteed total ordering, which makes the implementations of
                // `PartialEq` and `Ord` consistent.
                self.position.cmp(&other.position)
            })
        })
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

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
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
        // Notice that we flip the ordering on costs for f-scores, to convert the queue into a min-heap
        // instead of a max-heap.
        compare_option_scores(other.f_score, self.f_score).then_with(|| {
            // If the f-scores are equal, then we should tie-break with their g-scores;
            // this doesn't affect optimality, but it's a well-known optimization in the
            // literature to prefer the node with the higher g-score, since that one is almost
            // certainly closer to the target (same f && higher g -> lower h).
            compare_option_scores(self.g_score, other.g_score).then_with(|| {
                // If the f and g scores are all equal, then we compare the positions of the nodes;
                // this gives us a guaranteed total ordering, which makes the implementations of
                // `PartialEq` and `Ord` consistent.
                self.position.cmp(&other.position)
            })
        })
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

/// Handles node expansion into the open set for graph-search A*.
///
/// `start` is the node that is currently being expanded
/// `neighbors` is the new nodes being considered for addition to the open set, as well as their f and g scores
/// `heap` this is the open set of nodes that are candidates for exploration/expansion
/// `parents` this is a lookup table from nodes to their parents
fn expand_neighbors<T: AStarNode, O: AStarCost>(
    start: T,
    neighbors: &[(T, O, O)],
    heap: &mut BinaryHeap<State<T, O>>,
    parents: &mut HashMap<T, T>,
    best_g_scores: &mut HashMap<T, O>,
    best_f_scores: &mut HashMap<T, O>,
) {
    for (neighbor, next_f_score, next_g_score) in neighbors {
        // Check if the g_score for the neighbor from this node is strictly less than
        // (i.e. better than) the current-best g_score for the neighbor; if not, skip it
        let neighbor_gscore_entry = best_g_scores.entry(*neighbor);
        
        let skip_neighbor: bool = match neighbor_gscore_entry {
            Entry::Vacant(_) => false,
            Entry::Occupied(v) => {
                if next_g_score < v.get() {
                    false
                } else {
                    true
                }
            },
        };

        if skip_neighbor {
            continue;
        }

        // If we're here, then the neighbor is better than the existing entry and should be put onto the open set
        let _ = parents.insert(*neighbor, start);

        let _ = best_g_scores.insert(*neighbor, *next_g_score);
        let _ = best_f_scores.insert(*neighbor, *next_f_score);

        heap.push(State {
            g_score: *next_g_score,
            f_score: *next_f_score,
            position: *neighbor,
        });
    }
}

/// Handles node expansion into the open set for grid-search A*.
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
    lowest_seen_g_scores: &mut HashMap<T, O>,
    parents: &mut HashMap<T, T>,
) where
    G: Fn(T, T) -> Option<O>,
    F: Fn(T) -> O,
    O: AStarCost,
{
    // let debug_print = true;
    // if (start.x.u8() == 25) & (start.y.u8() == 10) {
    //     // Divergence node: RoomXY { x: RoomCoordinate(25), y: RoomCoordinate(10) },
    //     println!("Expanding neighbors for divergence node");
    //     debug_print = true;
    // }
    // println!("Expanding neighbors for: {:?}", start);

    for (neighbor, direction) in neighbors {
        // Do this as a match statement, with this as the branch for vacant; the occupied branch
        // needs to do g-score checking, and then if the g-score is smaller than the lowest seen,
        // add that node to the heap
        if let Some(next_tile_cost) = cost_fn(start, *neighbor) {
            if let Some(next_g_score) = g_score.checked_add(&next_tile_cost) {
                let add_state_to_heap: bool = match parents.entry(*neighbor) {
                    Entry::Occupied(mut v) => {
                        // Check the g_score of the neighbor we're examining against the smallest
                        // we've seen for that node; if the current g_score is smaller, then we
                        // want to add an updated copy of the node to the heap with the new
                        // g_score.
                        //
                        // We *could* modify the existing entry in the heap, but that would require
                        // us to convert the heap into an iterable, modify the entry there, and
                        // rebuild the heap. For a once-in-a-blue-moon thing, not terrible, but
                        // this can happen regularly, so we want to avoid all that work. The
                        // trade-off is that we have to filter those higher g_score nodes when
                        // we're popping those node entries off of the heap to explore the
                        // frontier.
                        match lowest_seen_g_scores.entry(*neighbor) {
                            Entry::Vacant(score_entry) => {
                                // This should never happen, but for safety and sanity, just add the
                                // node to the frontier heap anyway.
                                score_entry.insert(next_g_score);
                                v.insert(start);
                                true
                            }
                            Entry::Occupied(mut score_entry) => {
                                if next_g_score < *score_entry.get() {
                                    // We've arrived at the node via a shorter path, adjust its
                                    // lowest score entry and parent pointer
                                    let _ = score_entry.insert(next_g_score);
                                    let _ = v.insert(start);
                                    true
                                } else {
                                    // We've arrived at the node via a same-length or longer path,
                                    // this can't ever be part of a shorter path, don't add it to
                                    // the frontier
                                    false
                                }
                            }
                        }
                    }
                    Entry::Vacant(v) => {
                        // We've never seen this node before, add it to the frontier
                        lowest_seen_g_scores.insert(*neighbor, next_g_score);
                        v.insert(start);
                        true
                    }
                };

                if add_state_to_heap {
                    let raw_h_score = heuristic_fn(*neighbor);
                    let h_score = raw_h_score;
                    if let Some(f_score) = next_g_score.checked_add(&h_score) {
                        // if debug_print {
                        //     println!("Node: {:?}, g: {:?}, h: {:?}, f: {:?}", neighbor, next_g_score, h_score, f_score);
                        // }

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
                }
            } else {
                // The g_score cost add has saturated, skip this neighbor
                continue;
            }
        } else {
            // No cost function output, skip this neighbor
            continue;
        }
    }
}

// fn generate_neighbors_direction_flatten_heuristic<T, H, O>(start: T, raw_node_heuristic: H) -> impl Fn(T) -> O
// where
//   T: AStarGridNode,
//   H: Fn(T) -> O,
//   O: AStarCost + std::ops::Sub<u32, Output = O>,
// {
//     let all_directions_iter = Direction::iter();
//     let diagonal_neighbors: Vec<T> = all_directions_iter
//         .filter(|d| d.is_diagonal())
//         .flat_map(|d| start.checked_add_direction(*d))
//         .collect();
//
//     move |node: T| {
//         // We always need to calculate the f_score for a node
//         let raw_f_score = raw_node_heuristic(node);
//         match diagonal_neighbors.contains(|n| *n == node) {
//             true => raw_f_score - 1, // If we find the node is a diagonal move, return the f_score slightly downgraded
//             false => raw_f_score, // The node isn't a diagonal move, so return the f_score unchanged
//         }
//     }
// }

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
///
/// Note: This function assumes that the heuristic function is consistent, and optimizes
/// accordingly. If your heuristic function is admissible but not consistent, then you risk getting
/// suboptimal paths, just like if your heuristic function is not admissible.
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

    // This is a tree of parent pointers, used to reconstruct the final path
    let mut parents: HashMap<T, T> = HashMap::new();

    // This is the open set, the frontier of nodes we have yet to explore in our search
    let mut heap = BinaryHeap::new();

    // This is a lookup table of the lowest g-score we've seen for any particular node, used to
    // determine whether we need to re-explore a node or not
    let mut lowest_seen_g_scores: HashMap<T, O> = HashMap::new();

    for s in start.iter().copied() {
        let initial_open_entry = GridState {
            g_score: Some(initial_cost),
            f_score: Some(heuristic_fn(s)),
            position: s,
            open_direction: None,
        };
        heap.push(initial_open_entry);

        lowest_seen_g_scores.insert(s, initial_cost);
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

        let should_skip_node: bool = match lowest_seen_g_scores.entry(position) {
            Entry::Vacant(_) => {
                // This should never happen, but for safety and sanity, expand the node
                false
            }
            Entry::Occupied(score_entry) => {
                if g_score > *score_entry.get() {
                    // If the g-score for this node is higher than the lowest that we've seen, then
                    // this is part of a suboptimal path and should be skipped
                    true
                } else {
                    false
                }
            }
        };

        if should_skip_node {
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
        if best_reached_f_score.is_none_or(|brfs| f_score < brfs) {
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

        let neighbors: Vec<(T, Direction)> = directions
            .iter()
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
            &mut lowest_seen_g_scores,
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

/// Highly-generic implementation of A* search algorithm on a graph.
///
/// Allows multiple starting nodes, a generic goal function,
/// generic cost and heuristic functions, control over
/// the maximum operations performed while searching,
/// and the maximum travel cost the final path can have.
///
/// Parameters:
///
/// - `start`: A slice of the starting nodes for the search
/// - `goal_fn`: A predicate function that takes a node and returns true if the node is a goal node
/// - `neighbors_fn`: A function that takes in the current node and its g score and returns its neighbors, as well as their f and g scores
/// - `max_ops`: The maximum number of expand operations to perform before stopping search
/// - `max_cost`: The maximum cost to allow for the final path before stopping search
/// - `initial_cost`: The initial cost to start the search with; this is the g_score
/// - `initial_estimate`: The initial estimate to start the search with; this is the f_score
///
/// Note: This function assumes that the heuristic function is consistent, and optimizes
/// accordingly. If your heuristic function is admissible but not consistent, then you risk getting
/// suboptimal paths, just like if your heuristic function is not admissible.
pub fn shortest_path_generic_graph<T: AStarNode + std::fmt::Debug + std::fmt::Display, P, N, O>(
    start: &[T],
    goal_fn: &P,
    neighbors_fn: &N,
    max_ops: u32,
    max_cost: O,
    initial_cost: O,
    initial_estimate: O,
) -> AStarSearchResults<T, O>
where
    P: Fn(T) -> bool,
    N: Fn(T, O) -> Vec<(T, O, O)>,
    O: AStarCost,
{
    let mut remaining_ops: u32 = max_ops;
    let mut best_reached = start[0];
    let mut best_reached_f_score = initial_estimate;

    let mut parents: HashMap<T, T> = HashMap::new();
    let mut heap = BinaryHeap::new();
    let mut best_g_scores: HashMap<T, O> = HashMap::new();
    let mut best_f_scores: HashMap<T, O> = HashMap::new();

    for s in start.iter().copied() {
        let _ = best_g_scores.insert(s, initial_cost);
        let _ = best_f_scores.insert(s, initial_estimate);
        let initial_open_entry = State {
            g_score: initial_cost,
            f_score: initial_estimate,
            position: s,
        };
        heap.push(initial_open_entry);
    }
    
    // Examine the frontier with lower cost nodes first (min-heap)
    while let Some(State {
        g_score,
        f_score,
        position,
    }) = heap.pop()
    {
        // We found the goal state, return the search results
        if goal_fn(position) {
            let path_opt = get_path_from_parents(&parents, position);
            return AStarSearchResults {
                ops_used: max_ops - remaining_ops,
                cost: Some(g_score),
                incomplete: false,
                path: path_opt.unwrap_or_else(|| Vec::new()),
            };
        }

        // Don't evaluate this state entry if its g score is higher than the best g score we've
        // encountered for this node
        if let Some(v) = best_g_scores.get(&position) {
            if g_score > *v {
                continue;
            }
        }

        // Don't evaluate children if we're beyond the maximum cost
        if g_score >= max_cost {
            continue;
        }

        // if this is the most promising path yet, mark it as the point to use for rebuild
        // if we have to return incomplete
        // Safety: we can unwrap here because we already know it's not None
        if f_score < best_reached_f_score {
            best_reached = position;
            best_reached_f_score = f_score;
        }

        remaining_ops -= 1;

        // Stop searching if we've run out of remaining ops we're allowed to perform
        if remaining_ops == 0 {
            break;
        }

        let neighbors: Vec<(T, O, O)> = neighbors_fn(position, g_score);

        expand_neighbors(position, &neighbors, &mut heap, &mut parents, &mut best_g_scores, &mut best_f_scores);
    }
    
    // Goal not reachable
    let path_opt = get_path_from_parents(&parents, best_reached);
    AStarSearchResults {
        ops_used: max_ops - remaining_ops,
        cost: Some(best_reached_f_score),
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
///
/// Note: This function assumes that the heuristic function is consistent, and optimizes
/// accordingly. If your heuristic function is admissible but not consistent, then you risk getting
/// suboptimal paths, just like if your heuristic function is not admissible.
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
    shortest_path_roomxy_multistart(&[start], goal_fn, cost_fn, heuristic_fn)
}

/// Convenience method for running multi-start A* with default costs
/// while still retaining control over the heuristic function used.
///
/// Note: This function assumes that the heuristic function is consistent, and optimizes
/// accordingly. If your heuristic function is admissible but not consistent, then you risk getting
/// suboptimal paths, just like if your heuristic function is not admissible.
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
///
/// Note: This function assumes that the heuristic function is consistent, and optimizes
/// accordingly. If your heuristic function is admissible but not consistent, then you risk getting
/// suboptimal paths, just like if your heuristic function is not admissible.
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
    shortest_path_position_multistart(&[start], goal_fn, cost_fn, heuristic_fn)
}

/// Convenience method for running multi-start A* with default costs
/// while still retaining control over the heuristic function used.
///
/// Note: This function assumes that the heuristic function is consistent, and optimizes
/// accordingly. If your heuristic function is admissible but not consistent, then you risk getting
/// suboptimal paths, just like if your heuristic function is not admissible.
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

#[allow(dead_code)]
fn optionize_cost_fn_results<G, T, O>(cost_fn: G) -> impl Fn(T, T) -> Option<O>
where
    G: Fn(T, T) -> O,
{
    move |a, b| Some(cost_fn(a, b))
}

#[cfg(test)]
mod helper_tests;

#[cfg(test)]
mod generic_grid_tests;

#[cfg(test)]
mod generic_graph_tests;
