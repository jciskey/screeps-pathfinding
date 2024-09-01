use screeps::{Direction, Position, RoomXY};

/// Trait that encapsulates taking a [screeps::Direction] constant and
/// producing a new node from it.
pub trait AddDirection {
    fn checked_add_direction(self, direction: Direction) -> Option<Self>
    where
        Self: Sized;
}

impl AddDirection for RoomXY {
    fn checked_add_direction(self, direction: Direction) -> Option<Self> {
        Self::checked_add_direction(self, direction)
    }
}

impl AddDirection for Position {
    fn checked_add_direction(self, direction: Direction) -> Option<Self> {
        Self::checked_add_direction(self, direction).ok()
    }
}

/// Trait that encapsulates being able to get a standardized range value
/// from one node to another.
pub trait GetRangeTo {
    fn get_range_to(self, other: Self) -> u32;
}

impl GetRangeTo for RoomXY {
    fn get_range_to(self, other: Self) -> u32 {
        self.get_range_to(other).into()
    }
}

impl GetRangeTo for Position {
    fn get_range_to(self, other: Self) -> u32 {
        self.get_range_to(other)
    }
}
