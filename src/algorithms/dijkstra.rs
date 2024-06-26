// https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

// Sample code pulled (and modified) from: https://doc.rust-lang.org/nightly/std/collections/binary_heap/index.html#examples

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::hash::Hash;

/// A simple trait encapsulating what other traits are needed
/// for a type to be usable in Dijkstra's Algorithm.
pub trait DijkstraNode: Eq + Hash + Copy + Ord {}
impl<T> DijkstraNode for T where T: Eq + Hash + Copy + Ord {}

#[derive(Debug)]
pub struct DijkstraSearchResults<T>
where
    T: DijkstraNode,
{
    ops_used: u32,
    cost: u32,
    incomplete: bool,
    path: Vec<T>,
}

impl<T: DijkstraNode> DijkstraSearchResults<T> {
    pub fn ops(&self) -> u32 {
        self.ops_used
    }

    pub fn cost(&self) -> u32 {
        self.cost
    }

    pub fn incomplete(&self) -> bool {
        self.incomplete
    }

    pub fn path(&self) -> &[T] {
        &self.path
    }
}

#[derive(Copy, Clone, Eq, PartialEq)]
struct State<T>
where
    T: Ord,
{
    cost: u32,
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
            .cost
            .cmp(&self.cost)
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

/// Calculates a shortest path from `start` to `goal` using Dijkstra's Algorithm.
///
/// The algorithm itself doesn't care what type the nodes are, as long as you provide
/// a cost function that can convert a node of that type into a u32 cost to move
/// to that node, and a neighbors function that can generate a slice of nodes to explore.
///
/// The cost function should return u32::MAX for unpassable tiles.
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
/// let costs = screeps_pathfinding::utils::get_lcm_from_terrain(&room_terrain, plain_cost, swamp_cost);
/// let costs_fn = screeps_pathfinding::utils::movement_costs_from_lcm(&costs);
/// let neighbors_fn = screeps_pathfinding::utils::room_xy_neighbors;
/// let max_ops = 2000;
/// let max_cost = 2000;
///
/// let search_results = screeps_pathfinding::algorithms::dijkstra::shortest_path_generic(
///     start,
///     goal,
///     costs_fn,
///     neighbors_fn,
///     max_ops,
///     max_cost,
/// );
///
/// if !search_results.incomplete() {
///   let path = search_results.path();
///   println!("Path: {:?}", path);
/// }
/// else {
///   println!("Could not find Dijkstra shortest path.");
///   println!("Search Results: {:?}", search_results);
/// }
/// ```
///
/// Reference: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
pub fn shortest_path_generic<T: DijkstraNode, G, N, I>(
    start: T,
    goal: T,
    cost_fn: G,
    neighbors: N,
    max_ops: u32,
    max_cost: u32,
) -> DijkstraSearchResults<T>
where
    G: Fn(T) -> u32,
    N: Fn(T) -> I,
    I: IntoIterator<Item = T, IntoIter: Iterator<Item = T>>,
{
    // Start at `start` and use `dist` to track the current shortest distance
    // to each node. This implementation isn't memory-efficient as it may leave duplicate
    // nodes in the queue. It also uses `u32::MAX` as a sentinel value,
    // for a simpler implementation.

    let mut remaining_ops: u32 = max_ops;
    let mut last_examined_cost: u32 = 0;

    // dist[node] = current shortest distance from `start` to `node`
    let mut dist: HashMap<T, u32> = HashMap::new();
    let mut parents: HashMap<T, T> = HashMap::new();

    let mut heap = BinaryHeap::new();

    // We're at `start`, with a zero cost
    dist.insert(start, 0);
    heap.push(State {
        cost: 0,
        position: start,
    });

    // Examine the frontier with lower cost nodes first (min-heap)
    while let Some(State { cost, position }) = heap.pop() {
        // We found the goal state, return the search results
        if position == goal {
            let path_opt = get_path_from_parents(&parents, start, position);
            return DijkstraSearchResults {
                ops_used: max_ops - remaining_ops,
                cost,
                incomplete: path_opt.is_none(),
                path: path_opt.unwrap_or_else(|| Vec::new()),
            };
        }

        remaining_ops -= 1;

        // Stop searching if we've run out of remaining ops we're allowed to perform
        if remaining_ops == 0 {
            break;
        }

        // Stop searching if our current cost is greater than what we're willing to pay
        // Note: Because our heap is sorted by cost, no later nodes can have a smaller
        // cost, so we can safely break here
        last_examined_cost = cost;
        if cost >= max_cost {
            break;
        }

        // Important as we may have already found a better way
        let current_cost = match dist.get(&position) {
            Some(c) => c,
            None => &u32::MAX,
        };
        if cost > *current_cost {
            continue;
        }

        // For each node we can reach, see if we can find a way with
        // a lower cost going through this node
        for p in neighbors(position) {
            let next_tile_cost = cost_fn(p);

            // u32::MAX is our sentinel value for unpassable, skip this neighbor
            if next_tile_cost == u32::MAX {
                continue;
            }

            let next = State {
                cost: cost + next_tile_cost,
                position: p,
            };

            // If so, add it to the frontier and continue
            let current_next_cost = match dist.get(&next.position) {
                Some(c) => c,
                None => &u32::MAX,
            };
            if next.cost < *current_next_cost {
                heap.push(next);

                // Relaxation, we have now found a better way
                if let Some(c) = dist.get_mut(&next.position) {
                    *c = next.cost;
                } else {
                    dist.insert(next.position, next.cost);
                }

                if let Some(parent_node) = parents.get_mut(&next.position) {
                    *parent_node = position;
                } else {
                    parents.insert(next.position, position);
                }
            }
        }
    }

    // Goal not reachable
    DijkstraSearchResults {
        ops_used: max_ops - remaining_ops,
        cost: last_examined_cost,
        incomplete: true,
        path: Vec::new(),
    }
}

fn get_path_from_parents<T: DijkstraNode>(
    parents: &HashMap<T, T>,
    origin: T,
    end: T,
) -> Option<Vec<T>> {
    let mut path = Vec::new();

    let mut current_pos = end;

    path.push(end);

    while current_pos != origin {
        let parent = parents.get(&current_pos)?;
        path.push(*parent);
        current_pos = *parent;
    }

    Some(path.into_iter().rev().collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use screeps::constants::Direction;
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

    fn room_xy_neighbors(node: RoomXY) -> Vec<RoomXY> {
        node.neighbors()
    }

    fn position_neighbors(node: Position) -> Vec<Position> {
        Direction::iter()
            .filter_map(|dir| node.checked_add_direction(*dir).ok())
            .collect()
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
            start,
            goal,
            all_tiles_are_plains_costs,
            room_xy_neighbors,
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
            start,
            goal,
            all_tiles_are_plains_costs,
            position_neighbors,
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
            start,
            goal,
            roomxy_unreachable_tile_costs,
            room_xy_neighbors,
            2000,
            2000,
        );

        println!("{:?}", search_results);

        assert_eq!(search_results.incomplete(), true);
        assert_eq!(search_results.cost() > 0, true);
        assert_eq!(search_results.ops() == 2000, true);

        let path = search_results.path();

        assert_eq!(path.len(), 0);
    }

    #[test]
    fn unreachable_target_position() {
        let room_name = "E5N6";
        let start = new_position(room_name, 10, 10);
        let goal = new_position(room_name, 10, 12);
        let search_results = shortest_path_generic(
            start,
            goal,
            position_unreachable_tile_costs,
            position_neighbors,
            2000,
            2000,
        );

        println!("{:?}", search_results);

        assert_eq!(search_results.incomplete(), true);
        assert_eq!(search_results.cost() > 0, true);
        assert_eq!(search_results.ops() == 2000, true);

        let path = search_results.path();

        assert_eq!(path.len(), 0);
    }

    #[test]
    fn max_ops_halt_roomxy() {
        let max_ops_failure = 5;
        let max_ops_success = 100;
        let start = unsafe { RoomXY::unchecked_new(10, 10) };
        let goal = unsafe { RoomXY::unchecked_new(10, 12) }; // This target generally takes ~11 ops to find

        // Failure case
        let search_results = shortest_path_generic(
            start,
            goal,
            all_tiles_are_plains_costs,
            room_xy_neighbors,
            max_ops_failure,
            2000,
        );

        assert_eq!(search_results.incomplete(), true);
        assert_eq!(search_results.cost() > 0, true);
        assert_eq!(search_results.ops() == max_ops_failure, true);

        let path = search_results.path();

        assert_eq!(path.len(), 0);

        // Success case
        let search_results = shortest_path_generic(
            start,
            goal,
            all_tiles_are_plains_costs,
            room_xy_neighbors,
            max_ops_success,
            2000,
        );

        assert_eq!(search_results.incomplete(), false);
        assert_eq!(search_results.cost() > 0, true);
        assert_eq!(search_results.ops() < max_ops_success, true);

        let path = search_results.path();

        assert_eq!(path.len(), 3);
    }

    #[test]
    fn max_ops_halt_position() {
        let max_ops_failure = 5;
        let max_ops_success = 100;
        let room_name = "E5N6";
        let start = new_position(room_name, 10, 10);
        let goal = new_position(room_name, 10, 12); // This target generally takes ~11 ops to find

        // Failure case
        let search_results = shortest_path_generic(
            start,
            goal,
            all_tiles_are_plains_costs,
            position_neighbors,
            max_ops_failure,
            2000,
        );

        assert_eq!(search_results.incomplete(), true);
        assert_eq!(search_results.cost() > 0, true);
        assert_eq!(search_results.ops() == max_ops_failure, true);

        let path = search_results.path();

        assert_eq!(path.len(), 0);

        // Success case
        let search_results = shortest_path_generic(
            start,
            goal,
            all_tiles_are_plains_costs,
            position_neighbors,
            max_ops_success,
            2000,
        );

        assert_eq!(search_results.incomplete(), false);
        assert_eq!(search_results.cost() > 0, true);
        assert_eq!(search_results.ops() < max_ops_success, true);

        let path = search_results.path();

        assert_eq!(path.len(), 3);
    }

    #[test]
    fn max_cost_halt_roomxy() {
        let max_cost_failure = 5;
        let max_cost_success = 100;
        let start = unsafe { RoomXY::unchecked_new(10, 10) };
        let goal = unsafe { RoomXY::unchecked_new(10, 12) }; // This target will cost 10 to move to

        // Failure case
        let search_results = shortest_path_generic(
            start,
            goal,
            all_tiles_are_swamps_costs,
            room_xy_neighbors,
            2000,
            max_cost_failure,
        );

        assert_eq!(search_results.incomplete(), true);
        assert_eq!(search_results.cost() >= max_cost_failure, true);
        assert_eq!(search_results.ops() < 2000, true);

        let path = search_results.path();

        assert_eq!(path.len(), 0);

        // Success case
        let search_results = shortest_path_generic(
            start,
            goal,
            all_tiles_are_swamps_costs,
            room_xy_neighbors,
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
            start,
            goal,
            all_tiles_are_swamps_costs,
            position_neighbors,
            2000,
            max_cost_failure,
        );

        assert_eq!(search_results.incomplete(), true);
        assert_eq!(search_results.cost() >= max_cost_failure, true);
        assert_eq!(search_results.ops() < 2000, true);

        let path = search_results.path();

        assert_eq!(path.len(), 0);

        // Success case
        let search_results = shortest_path_generic(
            start,
            goal,
            all_tiles_are_swamps_costs,
            position_neighbors,
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
