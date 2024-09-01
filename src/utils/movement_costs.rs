
use std::rc::Rc;
use std::cell::RefCell;

use screeps::constants::Terrain;
use screeps::local::{LocalCostMatrix, LocalRoomTerrain};
use screeps::{Position, RoomName, RoomXY};

use super::cache::{LCMCache, TerrainCache};

pub fn get_movement_cost_lcm_from_terrain(
    room_terrain: &LocalRoomTerrain,
    plain_cost: u8,
    swamp_cost: u8,
) -> LocalCostMatrix {
    let mut cm = LocalCostMatrix::new();

    for (xy, val) in cm.iter_mut() {
        *val = match room_terrain.get_xy(xy) {
            Terrain::Wall => u8::MAX,
            Terrain::Plain => plain_cost,
            Terrain::Swamp => swamp_cost,
        }
    }

    cm
}

/// Builds a cost function closure that pulls costs from a `LocalCostMatrix`.
///
/// # Example
/// ```rust
/// use screeps::{LocalRoomTerrain, Position, RoomCoordinate, RoomXY};
///
/// let room_terrain = LocalRoomTerrain::new_from_bits(Box::new([0; 2500])); // Terrain that's all plains
/// let plain_cost = 1;
/// let swamp_cost = 5;
/// let costs = screeps_pathfinding::utils::movement_costs::get_movement_cost_lcm_from_terrain(&room_terrain, plain_cost, swamp_cost);
///
/// let roomxy_costs_fn = screeps_pathfinding::utils::movement_costs::movement_costs_from_lcm(&costs);
///
/// let xy = RoomXY::checked_new(24, 18).unwrap();
/// assert_eq!(roomxy_costs_fn(xy), plain_cost as u32);
///
/// let position_costs_fn = screeps_pathfinding::utils::movement_costs::movement_costs_from_lcm(&costs);
///
/// let pos = Position::new(
///     RoomCoordinate::try_from(24).unwrap(),
///     RoomCoordinate::try_from(18).unwrap(),
///     "E5N6".parse().unwrap(),
/// );
/// assert_eq!(position_costs_fn(pos), plain_cost as u32);
/// ```
pub fn movement_costs_from_lcm<T>(lcm: &LocalCostMatrix) -> impl Fn(T) -> u32 + '_
where
    T: Into<RoomXY>,
{
    |xy| {
        let value = lcm.get(xy.into());
        match value {
            u8::MAX => u32::MAX,
            _ => value as u32,
        }
    }
}

/// Utility function to create an LCM generation closure from a terrain cache and terrain costs.
pub fn get_lcm_generation_closure_from_terrain_cache(
    terrain_cache_ref: Rc<RefCell<TerrainCache>>,
    plain_cost: u8,
    swamp_cost: u8,
    default_cost: u8,
) -> impl Clone + Fn(&RoomName) -> LocalCostMatrix {
    move |room_name: &RoomName| {
        let mut cm = LocalCostMatrix::new_with_value(default_cost);

        if let Ok(mut terrain_cache) = terrain_cache_ref.try_borrow_mut() {
            if let Some(room_terrain) = terrain_cache.get_terrain(room_name) {
                for (xy, val) in cm.iter_mut() {
                    *val = match room_terrain.get_xy(xy) {
                        Terrain::Wall => u8::MAX,
                        Terrain::Plain => plain_cost,
                        Terrain::Swamp => swamp_cost,
                    }
                }
            }
        }

        cm
    }
}

/// Utility function to create a movement costs closure from an LCM cache.
pub fn movement_costs_from_lcm_cache<'a, 'b, F: Clone + Fn(&RoomName) -> LocalCostMatrix + 'a + 'b>(
    lcm_cache_ref: &'b RefCell<LCMCache>,
    generator_fn: F
) -> impl Fn(Position) -> u32 + 'b {
    move |pos: Position| {
        let mut lcm_cache = lcm_cache_ref.borrow_mut();
        let lcm = lcm_cache.get_lcm(&pos.room_name(), generator_fn.clone());
        let value = lcm.get(pos.into());
        match value {
            u8::MAX => u32::MAX,
            _ => value as u32,
        }
    }
}