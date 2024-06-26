use screeps::constants::{Direction, Terrain};
use screeps::local::{LocalCostMatrix, LocalRoomTerrain};
use screeps::{Position, RoomXY};

pub fn get_lcm_from_terrain(
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

pub fn movement_costs_from_lcm(lcm: &LocalCostMatrix) -> impl Fn(RoomXY) -> u32 + '_ {
    |xy| {
        let value = lcm.get(xy);
        match value {
            u8::MAX => u32::MAX,
            _ => value as u32,
        }
    }
}
