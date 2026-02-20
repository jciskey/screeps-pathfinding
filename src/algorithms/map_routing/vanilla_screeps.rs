
use itertools::Itertools;

use screeps::{
    RoomName,
    constants::ExitDirection,
    game::map::RouteStep,
};

use crate::algorithms::astar::shortest_path_generic_graph;

#[derive(Debug)]
pub enum GameMapFindRouteError {
    NoPathFound,
    NonAdjacentRoomsInPathfinding,
}

// Notes:
// 
// Original Screeps engine algorithm uses A* search across the map.
// The exits are just based on if there's an exit at all; base room connectivity.
// Pulls in the exit data directly from the preloaded runtime game data.


// Algorithm structure
//
// We're trying to mimic the baseline game algorithm, so we're going to use A* across a basic room
// connectivity graph.
//
// Parameters will be:
// - Starting room
// - Target room
// - Room neighbors fn
//   - A function that takes in the current RoomName and its g score and returns its neighbors, as well as their f and g scores
// 
// Final output route should be a Vec<(ExitDirection, RoomName)>.
// If the start and goal rooms are the same room, the returned Vec should be empty.
// If there is no route, return None.
//
// With the addition of implementing the AddDirection trait in common/traits, RoomName implements
// the other necessary traits to get AStarNode auto-implemented, no extra work required on our end.
// We can just start using the existing A* we've already implemented.

/// Re-implements the game map find route method in Rust with A*, making calls to JS to
/// pull the exits for each room.
///
/// `neighbors` should be a function that takes in a [`RoomName`] and returns a Vec of 3-tuples,
/// containing (in order), the neighbor RoomName, the f_score of that neighbor, and the g_score of
/// that neighbor.
pub fn game_map_find_route<N>(start: RoomName, goal: RoomName, neighbors: &N) -> Result<Vec<RouteStep>, GameMapFindRouteError>
where
N: Fn(RoomName, u32) -> Vec<(RoomName, u32, u32)>,
{
    // Edge case: start == goal
    if start == goal {
        return Ok(Vec::new());
    }

    let goal_fn = |node: RoomName| node == goal;

    let search_start = [start];
    let max_ops = 5000;
    let max_cost = 5000;
    let initial_cost = 0;
    let initial_estimate = screeps::game::map::get_room_linear_distance(start, goal, false);

    let search_results = shortest_path_generic_graph(
        &search_start,
        &goal_fn,
        neighbors,
        max_ops,
        max_cost,
        initial_cost,
        initial_estimate,
    );

    if search_results.incomplete() {
        Err(GameMapFindRouteError::NoPathFound)
    } else {
        let path = search_results.path();
        Ok(path.into_iter().copied()
            .tuple_windows()
            .map(|(a,b)| {
                let (dx, dy) = a - b;
                let exit = match (dx, dy) {
                    (0, 1) => ExitDirection::Top,
                    (-1, 0) => ExitDirection::Right,
                    (0, -1) => ExitDirection::Bottom,
                    (1, 0) => ExitDirection::Left,
                    _ => panic!("Should only ever get neighboring rooms"),
                };
                RouteStep { exit, room: b }
            })
            .collect()
        )
    }
}

pub fn room_neighbors_from_js(room: RoomName, g_score: u32) -> Vec<(RoomName, u32, u32)> {
    let new_g_score = g_score + 1;

    let js_hashmap = screeps::game::map::describe_exits(room);
    js_hashmap
        .entries()
        .map(|(_, name)| {
            let h = screeps::game::map::get_room_linear_distance(room, name, false);
            let f_score = new_g_score + h;
            (name, f_score, new_g_score)
        })
        .collect()
}
