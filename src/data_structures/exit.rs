use screeps::{ExitDirection, Position, RoomCoordinate, RoomName};


/// A room exit, independent of any particular room or edge.
#[derive(Debug, PartialEq)]
pub struct LocalExit {
    start: RoomCoordinate,
    length: u8,
}

/// Validation errors returned when attempting to create a [LocalExit].
#[derive(Debug, PartialEq)]
pub enum LocalExitCreateError {
    LengthTooShort,
    LengthTooLong,
    CoordinateAtCorner,
}

impl LocalExit {
    /// Create a [LocalExit] from a [RoomCoordinate] and a [u8] length.
    ///
    /// Returns an error if:
    /// - start.u8() == 0 or 49
    /// - length is 0
    /// - length > 48
    ///   - The first possible exit tile is 1
    ///   - The last possible exit tile is 48
    ///   - Thus, the maximum length possible is 48
    /// - start.u8() + length > 49
    pub fn new(start: RoomCoordinate, length: u8) -> Result<Self, LocalExitCreateError> {
        match length {
            0 => Err(LocalExitCreateError::LengthTooShort),
            49.. => Err(LocalExitCreateError::LengthTooLong),
            _ => {
                let start_u8 = start.u8();
                let max_length = 49 - start_u8;
                if length > max_length {
                    Err(LocalExitCreateError::LengthTooLong)
                } else {
                    match start_u8 {
                        0 | 49 => Err(LocalExitCreateError::CoordinateAtCorner),
                        _ => {
                            // Safety:
                            // - Length has been validated
                            // - Coordinate has been validated
                            unsafe { Ok(LocalExit::unchecked_new(start, length)) }
                        }
                    }
                }
            }
        }
    }

    /// Creates a [LocalExit] without checking safety boundaries for the length.
    ///
    /// Safety: You must verify that `length` is 47 or less, or other methods may cause a panic.
    pub unsafe fn unchecked_new(start: RoomCoordinate, length: u8) -> Self {
        Self {
            start,
            length,
        }
    }

    /// Returns whether a given [RoomCoordinate] is within this exit.
    pub fn contains(&self, coord: &RoomCoordinate) -> bool {
        let start = self.start;
        let end = self.end();

        (start <= *coord) & (*coord <= end)
    }

    /// Returns all of the [RoomCoordinates](RoomCoordinate) within this exit.
    pub fn get_all_coordinates(&self) -> Vec<RoomCoordinate> {
        let start_u8 = self.start.u8();
        let end_u8 = self.end().u8();

        (start_u8..=end_u8).map(|x| unsafe { RoomCoordinate::unchecked_new(x)  }).collect()
    }

    /// Returns landmark tiles within this exit.
    ///
    /// Landmarks are particular points of interest. See [LocalExitLandmarks] for details on which
    /// tiles are considered interesting.
    pub fn landmarks(&self) -> LocalExitLandmarks {
        let start = self.start;
        let end = self.end();
        let midpoint = self.midpoint();

        LocalExitLandmarks {
            start,
            end,
            midpoint,
        }
    }

    /// Returns the number of [RoomCoordinates](RoomCoordinate) within this exit.
    pub fn length(&self) -> u8 {
        self.length
    }

    /// Returns the first [RoomCoordinate] within this exit.
    pub fn start(&self) -> RoomCoordinate {
        self.start
    }

    /// Returns the last [RoomCoordinate] within this exit.
    pub fn end(&self) -> RoomCoordinate {
        // Safety: u8 -> i8 casting is okay, since length is always LTE 47, which is well-below
        // i8::MAX
        self.start.saturating_add((self.length - 1) as i8)
    }

    /// Returns the [RoomCoordinate] halfway between the first and last tiles within this exit.
    pub fn midpoint(&self) -> RoomCoordinate {
        // Safety:
        //
        // Since start and end are both RoomCoordinates, their u8 values are guaranteed to be
        // less than ROOM_SIZE, which is well below (u8::MAX / 2); this means we can safely add
        // them without doing any sort of overflow checking.
        //
        // Also, we know from how end is constructed that it can't ever be > 48, and start is
        // guaranteed to be <= 48 as well.
        unsafe { RoomCoordinate::unchecked_new((self.start.u8() + self.end().u8()) / 2) }
    }
}

/// A room exit, tied to a specific room.
#[derive(Debug, PartialEq)]
pub struct RoomExit {
    exit: LocalExit,
    room: RoomName,
    direction: ExitDirection,
}

/// Validation errors returned when attempting to create a [RoomExit].
#[derive(Debug, PartialEq)]
pub enum RoomExitCreateError {
    LengthTooShort,
    LengthTooLong,
    PositionAtCorner,
    PositionNotOnRoomEdge,
}

impl RoomExit {
    /// Create a [RoomExit] from a [RoomCoordinate], a [u8] length, a [RoomName], and an
    /// [ExitDirection].
    ///
    /// Returns an error if:
    /// - start.u8() == 0 or 49
    /// - length is 0
    /// - length > 48
    ///   - The first possible exit tile is 1
    ///   - The last possible exit tile is 48
    ///   - Thus, the maximum length possible is 48
    /// - start.u8() + length > 49
    pub fn new(room: RoomName, start: RoomCoordinate, length: u8, direction: ExitDirection) -> Result<Self, RoomExitCreateError> {
        let local_exit_res = LocalExit::new(start, length);
        match local_exit_res {
            Ok(local_exit) => {
                Ok(Self {
                    exit: local_exit,
                    room,
                    direction,
                })
            },
            Err(err) => {
                match err {
                    LocalExitCreateError::LengthTooShort => Err(RoomExitCreateError::LengthTooShort),
                    LocalExitCreateError::LengthTooLong => Err(RoomExitCreateError::LengthTooLong),
                    LocalExitCreateError::CoordinateAtCorner => Err(RoomExitCreateError::PositionAtCorner),
                }
            },
        }
    }

    /// Creates a [RoomExit] without checking safety boundaries for the length.
    ///
    /// Safety: You must verify that `length` is between 1 and 48, inclusive, or other methods may cause a panic.
    pub unsafe fn unchecked_new(room: RoomName, start: RoomCoordinate, length: u8, direction: ExitDirection) -> Self {
        let exit = unsafe { LocalExit::unchecked_new(start, length) };
        
        Self {
            exit,
            room,
            direction,
        }
    }

    /// Creates a [RoomExit] from a [Position], a length, and a [RoomName].
    ///
    /// The x and y coordinates of the Position determine what direction the exit is for. Exactly
    /// one of x and y must be 0 or 49, and the other must be in the inclusive range \[1,48\].
    ///
    /// Returns an error if:
    /// - position is not on an edge
    /// - position is on a corner
    /// - length > 48
    /// - length is 0
    /// - the exit would extend beyond x/y 49 (pos + length > 49)
    pub fn new_from_position(room: RoomName, pos: Position, length: u8) -> Result<Self, RoomExitCreateError> {
        match get_direction_and_coordinate_from_position(pos) {
            Ok((dir, start)) => {
                let local_exit_res = LocalExit::new(start, length);
                match local_exit_res {
                    Ok(local_exit) => {
                        Ok(Self {
                            exit: local_exit,
                            room,
                            direction: dir,
                        })
                    },
                    Err(err) => {
                        match err {
                            LocalExitCreateError::LengthTooShort => Err(RoomExitCreateError::LengthTooShort),
                            LocalExitCreateError::LengthTooLong => Err(RoomExitCreateError::LengthTooLong),
                            LocalExitCreateError::CoordinateAtCorner => Err(RoomExitCreateError::PositionAtCorner),
                        }
                    },
                }
            },
            Err(error) => Err(error),
        }
    }

    /// Returns whether a given [Position] is within this exit.
    pub fn contains(&self, pos: &Position) -> bool {
        if self.room != pos.room_name() {
            // Gotta be in the same room to be in the exit
            false
        } else {
            let res = get_direction_and_coordinate_from_position(*pos);
            if let Ok((dir, coord)) = res {
                if self.direction != dir {
                    // Gotta be on the same edge to be contained in the exit
                    false
                } else {
                    // Since we know the position is in the same room and is on the same edge, now
                    // it's just a matter of checking the relative position along the edge, which
                    // is what LocalExit::contains does.
                    self.exit.contains(&coord)
                }
            } else {
                // Any kind of error from get_direction_and_coordinate_from_position means the
                // provided position wasn't a valid edge tile, and thus can't be in the exit
                false
            }
        }
    }

    /// Returns all of the [Positions](Position) within this exit.
    pub fn get_all_positions(&self) -> Vec<Position> {
        let exit_iter = self.exit.get_all_coordinates().into_iter();

        let xy_iter = exit_iter.map(|v| match self.direction {
            ExitDirection::Top => (v, unsafe { RoomCoordinate::unchecked_new(0) }),
            ExitDirection::Right => (unsafe { RoomCoordinate::unchecked_new(49) }, v),
            ExitDirection::Bottom => (v, unsafe { RoomCoordinate::unchecked_new(49) }),
            ExitDirection::Left => (unsafe { RoomCoordinate::unchecked_new(0) }, v),
        });

        xy_iter
            .map(|(x, y)| Position::new(x, y, self.room))
            .collect()
    }

    /// Returns landmark tiles within this exit.
    ///
    /// Landmarks are particular points of interest. See [RoomExitLandmarks] for details on which
    /// tiles are considered interesting.
    pub fn landmarks(&self) -> RoomExitLandmarks {
        self.exit.landmarks().as_room_exit_landmarks(self.room, self.direction)
    }

    /// Returns the number of tiles within this exit.
    pub fn length(&self) -> u8 {
        self.exit.length()
    }

    /// Returns the first [Position] within this exit.
    pub fn start(&self) -> Position {
        get_position_from_coord_and_dir(self.room, self.exit.start(), self.direction)
    }

    /// Returns the last [Position] within this exit.
    pub fn end(&self) -> Position {
        get_position_from_coord_and_dir(self.room, self.exit.end(), self.direction)
    }

    pub fn midpoint(&self) -> Position {
        get_position_from_coord_and_dir(self.room, self.exit.midpoint(), self.direction)
    }
}

/// A collection of [RoomCoordinates](RoomCoordinate) within an exit that are typically useful.
///
/// Currently contains:
/// - The starting tile of the exit
/// - The ending tile of the exit
/// - The midpoint tile of the exit
///
/// In the case of exits with less than 3 total tiles, these values *can* overlap. Validate the
/// output appropriately.
pub struct LocalExitLandmarks {
    pub start: RoomCoordinate,
    pub end: RoomCoordinate,
    pub midpoint: RoomCoordinate,
}

/// A collection of [Positions](Position) within an exit that are typically useful.
///
/// Currently contains:
/// - The starting tile of the exit
/// - The ending tile of the exit
/// - The midpoint tile of the exit
///
/// In the case of exits with less than 3 total tiles, these values *can* overlap. Validate the
/// output appropriately.
pub struct RoomExitLandmarks {
    pub start: Position,
    pub end: Position,
    pub midpoint: Position,
}

impl LocalExitLandmarks {
    pub fn as_room_exit_landmarks(&self, room: RoomName, direction: ExitDirection) -> RoomExitLandmarks {
        let start = get_position_from_coord_and_dir(room, self.start, direction);
        let end = get_position_from_coord_and_dir(room, self.end, direction);
        let midpoint = get_position_from_coord_and_dir(room, self.midpoint, direction);

        RoomExitLandmarks {
            start,
            end,
            midpoint,
        }
    }
}

impl RoomExitLandmarks {
    pub fn as_local_exit_landmarks(&self) -> LocalExitLandmarks {
        // Safety: We know that start is a valid exit coordinate, so this will always return Ok
        let (direction, _) = get_direction_and_coordinate_from_position(self.start).unwrap();

        let (start, end, midpoint) = match direction {
            ExitDirection::Top | ExitDirection::Bottom => (self.start.x(), self.end.x(), self.midpoint.x()),
            ExitDirection::Right | ExitDirection::Left => (self.start.y(), self.end.y(), self.midpoint.y()),
        };

        LocalExitLandmarks {
            start,
            end,
            midpoint,
        }
    }
}

/// Determines the exit direction and edge coordinate for a position.
///
/// Returns an error if the position is not a valid exit position:
/// - Position is one of the 4 room corners
/// - Position is not on a room edge
pub fn get_direction_and_coordinate_from_position(pos: Position) -> Result<(ExitDirection, RoomCoordinate), RoomExitCreateError> {
    let x = pos.x();
    let y = pos.y();
    let x_u8 = x.u8();
    let y_u8 = y.u8();

    if (x_u8 != 0) & (x_u8 != 49) & (y_u8 != 0) & (y_u8 != 49) {
        Err(RoomExitCreateError::PositionNotOnRoomEdge)
    } else {
        match (x_u8, y_u8) {
            (0, 0) | (0, 49) | (49, 49) | (49, 0) => Err(RoomExitCreateError::PositionAtCorner),
            _ => {
                // At this point, we've caught any positions that weren't on the room edges
                // or were at the corners, meaning every possible pair of (x,y) values here
                // are valid potential exit tiles.
                let (dir, coord) = {
                    if y_u8 == 0 {
                        (ExitDirection::Top, x)
                    } else if x_u8 == 49 {
                        (ExitDirection::Right, y)
                    } else if y_u8 == 49 {
                        (ExitDirection::Bottom, x)
                    } else {
                        (ExitDirection::Left, y)
                    }
                };

                Ok((dir, coord))
            },
        }
    }
}

/// Converts a [RoomCoordinate] and an [ExitDirection] into a [Position] on the edge corresponding
/// to the provided exit direction.
///
/// Note: This is a generic conversion function. It does not have corner constraints, and will
/// absolutely return positions corresponding to each of the 4 corners.
pub fn get_position_from_coord_and_dir(room: RoomName, val: RoomCoordinate, direction: ExitDirection) -> Position {
    let (x, y) = match direction {
        ExitDirection::Top => (val, unsafe { RoomCoordinate::unchecked_new(0) }),
        ExitDirection::Right => (unsafe { RoomCoordinate::unchecked_new(49) }, val),
        ExitDirection::Bottom => (val, unsafe { RoomCoordinate::unchecked_new(49) }),
        ExitDirection::Left => (unsafe { RoomCoordinate::unchecked_new(0) }, val),
    };

    // Safety: One of x/y is a RoomCoordinate, and thus safe, and the other is 0/49, also safe
    Position::new(x, y, room)
}

