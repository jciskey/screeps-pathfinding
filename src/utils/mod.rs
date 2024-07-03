use screeps::constants::{Direction, Terrain};
use screeps::local::{LocalCostMatrix, LocalRoomTerrain};
use screeps::{Position, RoomXY};

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

pub fn room_xy_neighbors(node: RoomXY) -> Vec<RoomXY> {
    node.neighbors()
}

pub fn position_neighbors(node: Position) -> Vec<Position> {
    Direction::iter()
        .filter_map(|dir| node.checked_add_direction(*dir).ok())
        .collect()
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
/// let costs = screeps_pathfinding::utils::get_movement_cost_lcm_from_terrain(&room_terrain, plain_cost, swamp_cost);
///
/// let roomxy_costs_fn = screeps_pathfinding::utils::movement_costs_from_lcm(&costs);
///
/// let xy = RoomXY::checked_new(24, 18).unwrap();
/// assert_eq!(roomxy_costs_fn(xy), plain_cost as u32);
///
/// let position_costs_fn = screeps_pathfinding::utils::movement_costs_from_lcm(&costs);
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
