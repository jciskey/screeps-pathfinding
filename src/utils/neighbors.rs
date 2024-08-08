use screeps::constants::Direction;
use screeps::{Position, RoomXY};

pub fn room_xy_neighbors(node: RoomXY) -> Vec<RoomXY> {
    node.neighbors()
}

pub fn position_neighbors(node: Position) -> Vec<Position> {
    Direction::iter()
        .filter_map(|dir| node.checked_add_direction(*dir).ok())
        .collect()
}
