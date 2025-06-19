use std::iter;

use screeps::{ExitDirection, Position, RoomCoordinate, RoomName};
use super::exit::*;

// LocalExit::new returns Ok for valid length
#[test]
pub fn local_exit_returns_ok_for_valid_length() {
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            let res = LocalExit::new(coord, length);
            assert!(res.is_ok());
        }
    }
}

// LocalExit::new returns Err for invalid length
#[test]
pub fn local_exit_returns_err_for_invalid_length() {
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        let invalid_iter = iter::once(0).chain((max_length + 1)..=u8::MAX);
        for length in invalid_iter {
            let res = LocalExit::new(coord, length);
            assert!(res.is_err(), "x: {x}, length: {length}");
        }
    }
}


// LocalExit::contains returns true for all coordinates in the inclusive range (start, end)
// and false for all coordinates outside of that range
#[test]
pub fn local_exit_contains_returns_true_for_all_coordinates_in_the_exit() {
    let all_edge_coords: Vec<RoomCoordinate> = (0..=49).map(|v| RoomCoordinate::new(v).unwrap()).collect();
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            let exit = LocalExit::new(coord, length).unwrap();
            let returned = exit.get_all_coordinates();
            for coord in &all_edge_coords {
                assert_eq!(returned.contains(&coord), exit.contains(&coord));
            }
        }
    }
}


// LocalExit::get_all_coordinates returns all the coordinates in the inclusive range (start, end)
#[test]
pub fn local_exit_get_all_coordinates_returns_all_coordinates_in_the_exit() {
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            let end = x + (length - 1);
            let exit = LocalExit::new(coord, length).unwrap();
            let expected: Vec<RoomCoordinate> = (x..=end).map(|v| RoomCoordinate::new(v).unwrap()).collect();
            let returned = exit.get_all_coordinates();
            assert_eq!(expected, returned);
        }
    }
}


// LocalExit::length returns the length provided to the constructor
#[test]
pub fn local_exit_length_returns_provided_length() {
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            let exit = LocalExit::new(coord, length).unwrap();
            let returned = exit.length();
            assert_eq!(length, returned);
        }
    }
}


// LocalExit::start returns the RoomCoordinate provided to the constructor
#[test]
pub fn local_exit_start_returns_provided_coordinate() {
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            let exit = LocalExit::new(coord, length).unwrap();
            let returned = exit.start();
            assert_eq!(coord, returned);
        }
    }
}


// LocalExit::end returns the RoomCoordinate found from moving (length - 1) tiles along the exit
#[test]
pub fn local_exit_end_returns_correct_coordinate() {
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            let expected_end_u8 = x + length - 1;
            let expected_end = RoomCoordinate::new(expected_end_u8).unwrap();
            let exit = LocalExit::new(coord, length).unwrap();
            let returned = exit.end();
            assert_eq!(expected_end, returned);
        }
    }
}


// LocalExit::midpoint returns the RoomCoordinate found from the standard midpoint formula
#[test]
pub fn local_exit_midpoint_returns_correct_coordinate() {
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            let end_u8 = x + length - 1;
            let midpoint_u8 = (x + end_u8) / 2;
            let midpoint = RoomCoordinate::new(midpoint_u8).unwrap();
            let exit = LocalExit::new(coord, length).unwrap();
            let returned = exit.midpoint();
            assert_eq!(midpoint, returned, "x: {x}, length: {length}, end: {end_u8}");
        }
    }
}

const exit_directions: [ExitDirection; 4] = [
    ExitDirection::Top,
    ExitDirection::Right,
    ExitDirection::Bottom,
    ExitDirection::Left,
];

// RoomExit::new returns Ok for valid length
#[test]
pub fn room_exit_returns_ok_for_valid_length() {
    let room_name = RoomName::from_packed(0);
    let coord = RoomCoordinate::new(1).unwrap();
    for length in 1..=48 {
        for dir in exit_directions {
            let res = RoomExit::new(room_name, coord, length, dir);
            assert!(res.is_ok());
        }
    }
}


// RoomExit::new returns Err for invalid length
#[test]
pub fn room_exit_returns_err_for_invalid_length() {
    let room_name = RoomName::from_packed(0);
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        let invalid_iter = iter::once(0).chain((max_length + 1)..=u8::MAX);
        for length in invalid_iter {
            for dir in exit_directions {
                let res = RoomExit::new(room_name, coord, length, dir);
                assert!(res.is_err(), "x: 1, length: {length}, dir: {dir:?}");
            }
        }
    }
}


// RoomExit::new_from_position returns Ok for valid length and exit position
#[test]
pub fn room_exit_new_from_position_returns_ok_for_valid_length_exit_position() {
    let room_name = RoomName::from_packed(0);
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            for dir in exit_directions {
                let pos = get_position_from_coord_and_dir(room_name, coord, dir);
                let res = RoomExit::new_from_position(room_name, pos, length);
                assert!(res.is_ok());
            }
        }
    }
}


// RoomExit::new_from_position returns expected err for invalid length
#[test]
pub fn room_exit_new_from_position_returns_err_for_invalid_length() {
    let room_name = RoomName::from_packed(0);
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        let invalid_iter = iter::once(0).chain((max_length + 1)..=u8::MAX);
        for length in invalid_iter {
            for dir in exit_directions {
                let pos = get_position_from_coord_and_dir(room_name, coord, dir);
                let res = RoomExit::new_from_position(room_name, pos, length);
                assert!(res.is_err(), "x: {x}, length: {length}, direction: {dir:?}");

                let err = res.unwrap_err();
                match length {
                    0 => assert_eq!(RoomExitCreateError::LengthTooShort, err),
                    _ => assert_eq!(RoomExitCreateError::LengthTooLong, err),
                };
            }
        }
    }
}


// RoomExit::new_from_position returns expected err for corner positions
#[test]
pub fn room_exit_new_from_position_returns_err_for_corners() {
    let room_name = RoomName::from_packed(0);
    let coord_0 = RoomCoordinate::new(0).unwrap();
    let coord_49 = RoomCoordinate::new(49).unwrap();
    let corners = vec!(
        (coord_0, coord_0), // Top Left
        (coord_49, coord_0), // Top Right
        (coord_49, coord_49), // Bottom Right
        (coord_0, coord_49), // Bottom Left
    );
    for (x, y) in corners {
        let pos = Position::new(x, y, room_name);
        for length in 1..=48 {
            let res = RoomExit::new_from_position(room_name, pos, length);
            assert!(res.is_err(), "x: {x}, length: {length}");

            let err = res.unwrap_err();
            assert_eq!(RoomExitCreateError::PositionAtCorner, err);
        }
    }
}


// RoomExit::new_from_position returns expected err for non-edge positions
#[test]
pub fn room_exit_new_from_position_returns_err_for_nonedges() {
    let room_name = RoomName::from_packed(0);
    let non_edge_coords: Vec<RoomCoordinate> = (1..=48).map(|x| RoomCoordinate::new(x).unwrap()).collect();
    let non_edge_xy = itertools::iproduct!(non_edge_coords.clone(), non_edge_coords);
    for (x, y) in non_edge_xy {
        let pos = Position::new(x, y, room_name);
        for length in 1..=48 {
            let res = RoomExit::new_from_position(room_name, pos, length);
            assert!(res.is_err(), "x: {x}, length: {length}");

            let err = res.unwrap_err();
            assert_eq!(RoomExitCreateError::PositionNotOnRoomEdge, err);
        }
    }
}


// RoomExit::contains returns true for all positions in the inclusive range (start, end)
// and false for all positions not in that range.
#[test]
pub fn room_exit_contains_returns_true_for_included_positions() {
    let room_name = RoomName::from_packed(0);
    let all_coords: Vec<RoomCoordinate> = (0..=49).map(|x| RoomCoordinate::new(x).unwrap()).collect();
    let all_room_positions: Vec<Position> = itertools::iproduct!(all_coords.clone(), all_coords)
        .map(|(x, y)| Position::new(x, y, room_name))
        .collect();

    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            for dir in exit_directions {
                let pos = get_position_from_coord_and_dir(room_name, coord, dir);
                let exit = RoomExit::new_from_position(room_name, pos, length).unwrap();
                let all_exit_positions = exit.get_all_positions();
                
                for p in &all_room_positions {
                    assert_eq!(all_exit_positions.contains(p), exit.contains(p));
                }
            }
        }
    }

}


// RoomExit::get_all_positions returns all the positions in the inclusive range (start, end)
#[test]
pub fn room_exit_get_all_positions_returns_all_positions_in_the_exit() {
    let room_name = RoomName::from_packed(0);
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            for dir in exit_directions {
                let end = x + (length - 1);
                let pos = get_position_from_coord_and_dir(room_name, coord, dir);
                let exit = RoomExit::new_from_position(room_name, pos, length).unwrap();
                let expected: Vec<Position> = (x..=end)
                    .map(|v| RoomCoordinate::new(v).unwrap())
                    .map(|c| get_position_from_coord_and_dir(room_name, c, dir))
                    .collect();
                let returned = exit.get_all_positions();
                assert_eq!(expected, returned);
            }
        }
    }
}


// RoomExit::length returns the length provided to the constructor
#[test]
pub fn room_exit_length_returns_provided_length() {
    let room_name = RoomName::from_packed(0);
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            for dir in exit_directions {
                let pos = get_position_from_coord_and_dir(room_name, coord, dir);
                let exit = RoomExit::new_from_position(room_name, pos, length).unwrap();
                let returned = exit.length();
                assert_eq!(length, returned);
            }
        }
    }
}


// RoomExit::start returns the Position provided to the constructor
#[test]
pub fn room_exit_start_returns_provided_position() {
    let room_name = RoomName::from_packed(0);
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            for dir in exit_directions {
                let pos = get_position_from_coord_and_dir(room_name, coord, dir);
                let exit = RoomExit::new_from_position(room_name, pos, length).unwrap();
                let returned = exit.start();
                assert_eq!(pos, returned);
            }
        }
    }
}


// RoomExit::end returns the Position found from moving (length - 1) tiles along the exit
#[test]
pub fn room_exit_end_returns_expected_position() {
    let room_name = RoomName::from_packed(0);
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            let end_u8 = x + length - 1;
            let end_coord = RoomCoordinate::new(end_u8).unwrap();
            for dir in exit_directions {
                let pos = get_position_from_coord_and_dir(room_name, coord, dir);
                let exit = RoomExit::new_from_position(room_name, pos, length).unwrap();
                let expected_end = get_position_from_coord_and_dir(room_name, end_coord, dir);
                let returned = exit.end();
                assert_eq!(expected_end, returned);
            }
        }
    }
}


// RoomExit::midpoint returns the Position found from the standard midpoint formula
#[test]
pub fn room_exit_midpoint_returns_expected_position() {
    let room_name = RoomName::from_packed(0);
    for x in 1..=48 {
        let coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            let end_u8 = x + length - 1;
            let midpoint_u8 = (x + end_u8) / 2;
            let midpoint_coord = RoomCoordinate::new(midpoint_u8).unwrap();
            for dir in exit_directions {
                let pos = get_position_from_coord_and_dir(room_name, coord, dir);
                let exit = RoomExit::new_from_position(room_name, pos, length).unwrap();
                let expected_pos = get_position_from_coord_and_dir(room_name, midpoint_coord, dir);
                let returned = exit.midpoint();
                assert_eq!(expected_pos, returned);
            }
        }
    }
}


// LocalExitLandmarks converts to RoomExitLandmarks and back losslessly
#[test]
pub fn exit_landmarks_converts_to_from_other_exit_landmarks_losslessly() {
    let room_name = RoomName::from_packed(0);
    for x in 1..=48 {
        let start_coord = RoomCoordinate::new(x).unwrap();
        let max_length = 49 - x;
        for length in 1..=max_length {
            let end_u8 = x + length - 1;
            let end_coord = RoomCoordinate::new(end_u8).unwrap();
            let midpoint_u8 = (x + end_u8) / 2;
            let midpoint_coord = RoomCoordinate::new(midpoint_u8).unwrap();
            for dir in exit_directions {
                let start_pos = get_position_from_coord_and_dir(room_name, start_coord, dir);
                let end_pos = get_position_from_coord_and_dir(room_name, end_coord, dir);
                let midpoint_pos = get_position_from_coord_and_dir(room_name, midpoint_coord, dir);
                let exit = RoomExit::new_from_position(room_name, start_pos, length).unwrap();
                let returned = exit.landmarks();

                assert_eq!(start_pos, returned.start);
                assert_eq!(end_pos, returned.end);
                assert_eq!(midpoint_pos, returned.midpoint);

                // Convert to local exit landmarks
                let first_local_landmarks = returned.as_local_exit_landmarks();
                let converted_room_landmarks = first_local_landmarks.as_room_exit_landmarks(room_name, dir);
                let second_local_landmarks = converted_room_landmarks.as_local_exit_landmarks();
               
                // Test the conversion from room landmarks to local landmarks was lossless
                assert_eq!(first_local_landmarks.start, start_coord);
                assert_eq!(first_local_landmarks.end, end_coord);
                assert_eq!(first_local_landmarks.midpoint, midpoint_coord);
               
                // Test the conversion from room landmarks to local landmarks then back to room
                // landmarks was lossless
                assert_eq!(converted_room_landmarks.start, returned.start);
                assert_eq!(converted_room_landmarks.end, returned.end);
                assert_eq!(converted_room_landmarks.midpoint, returned.midpoint);
               
                // Test the conversion from local landmarks to room landmarks then back to local
                // landmarks was lossless
                assert_eq!(first_local_landmarks.start, second_local_landmarks.start);
                assert_eq!(first_local_landmarks.end, second_local_landmarks.end);
                assert_eq!(first_local_landmarks.midpoint, second_local_landmarks.midpoint);
            }
        }
    }
}


// get_direction_and_coordinate_from_position returns the expected ExitDirection and RoomCoordinate for positions along the edges
#[test]
pub fn get_direction_and_coordinate_from_position_returns_expected_direction_and_coordinate_for_non_corner_edge_positions() {
    let room_name = RoomName::from_packed(0);

    let coord_0 = RoomCoordinate::new(0).unwrap();
    let coord_49 = RoomCoordinate::new(49).unwrap();

    let top_edge_coords: Vec<_> = (1..=48).map(|x| (RoomCoordinate::new(x).unwrap(), coord_0)).collect();
    let right_edge_coords: Vec<_> = (1..=48).map(|y| (coord_49, RoomCoordinate::new(y).unwrap())).collect();
    let bottom_edge_coords: Vec<_> = (1..=48).map(|x| (RoomCoordinate::new(x).unwrap(), coord_49)).collect();
    let left_edge_coords: Vec<_> = (1..=48).map(|y| (coord_0, RoomCoordinate::new(y).unwrap())).collect();

    let top_edge_iter = itertools::izip!(top_edge_coords, std::iter::repeat(ExitDirection::Top));
    let right_edge_iter = itertools::izip!(right_edge_coords, std::iter::repeat(ExitDirection::Right));
    let bottom_edge_iter = itertools::izip!(bottom_edge_coords, std::iter::repeat(ExitDirection::Bottom));
    let left_edge_iter = itertools::izip!(left_edge_coords, std::iter::repeat(ExitDirection::Left));

    let all_edge_iters = itertools::chain!(top_edge_iter, right_edge_iter, bottom_edge_iter, left_edge_iter);

    for ((x, y), dir) in all_edge_iters {
        let pos = Position::new(x, y, room_name);
        let res = get_direction_and_coordinate_from_position(pos);
        assert!(res.is_ok());

        let expected_coord = match dir {
            ExitDirection::Top | ExitDirection::Bottom => x,
            ExitDirection::Right | ExitDirection::Left => y,
        };

        let (res_dir, res_coord) = res.unwrap();
        assert_eq!(dir, res_dir);
        assert_eq!(expected_coord, res_coord);
    }
}

// get_direction_and_coordinate_from_position returns the expected error for corner positions
#[test]
pub fn get_direction_and_coordinate_from_position_returns_err_for_corners() {
    let room_name = RoomName::from_packed(0);
    let coord_0 = RoomCoordinate::new(0).unwrap();
    let coord_49 = RoomCoordinate::new(49).unwrap();
    let corners = vec!(
        (coord_0, coord_0), // Top Left
        (coord_49, coord_0), // Top Right
        (coord_49, coord_49), // Bottom Right
        (coord_0, coord_49), // Bottom Left
    );
    for (x, y) in corners {
        let pos = Position::new(x, y, room_name);
        let res = get_direction_and_coordinate_from_position(pos);
        assert!(res.is_err());

        let err = res.unwrap_err();
        assert_eq!(RoomExitCreateError::PositionAtCorner, err);
    }
}


// get_direction_and_coordinate_from_position returns the expected error for non-edge positions
#[test]
pub fn get_direction_and_coordinate_from_position_returns_err_for_nonedges() {
    let room_name = RoomName::from_packed(0);
    let non_edge_coords: Vec<RoomCoordinate> = (1..=48).map(|x| RoomCoordinate::new(x).unwrap()).collect();
    let non_edge_xy = itertools::iproduct!(non_edge_coords.clone(), non_edge_coords);
    for (x, y) in non_edge_xy {
        let pos = Position::new(x, y, room_name);
        let res = get_direction_and_coordinate_from_position(pos);
        assert!(res.is_err());

        let err = res.unwrap_err();
        assert_eq!(RoomExitCreateError::PositionNotOnRoomEdge, err);
    }
}


// get_position_from_coord_and_dir returns an edge position as-expected
#[test]
pub fn get_position_from_coord_and_dir_returns_expected_position() {
    let room_name = RoomName::from_packed(0);

    let coord_0 = RoomCoordinate::new(0).unwrap();
    let coord_49 = RoomCoordinate::new(49).unwrap();

    let top_edge_coords: Vec<_> = (0..=49)
        .map(|x| (RoomCoordinate::new(x).unwrap(), coord_0))
        .map(|tpl| (tpl, tpl.0))
        .collect();
    let right_edge_coords: Vec<_> = (0..=49)
        .map(|y| (coord_49, RoomCoordinate::new(y).unwrap()))
        .map(|tpl| (tpl, tpl.1))
        .collect();
    let bottom_edge_coords: Vec<_> = (0..=49)
        .map(|x| (RoomCoordinate::new(x).unwrap(), coord_49))
        .map(|tpl| (tpl, tpl.0))
        .collect();
    let left_edge_coords: Vec<_> = (0..=49)
        .map(|y| (coord_0, RoomCoordinate::new(y).unwrap()))
        .map(|tpl| (tpl, tpl.1))
        .collect();

    let top_edge_iter = itertools::izip!(top_edge_coords, std::iter::repeat(ExitDirection::Top));
    let right_edge_iter = itertools::izip!(right_edge_coords, std::iter::repeat(ExitDirection::Right));
    let bottom_edge_iter = itertools::izip!(bottom_edge_coords, std::iter::repeat(ExitDirection::Bottom));
    let left_edge_iter = itertools::izip!(left_edge_coords, std::iter::repeat(ExitDirection::Left));

    let all_edge_iters = itertools::chain!(top_edge_iter, right_edge_iter, bottom_edge_iter, left_edge_iter);

    for (((x, y), coord), dir) in all_edge_iters {
        let pos = Position::new(x, y, room_name);
        let returned_pos = get_position_from_coord_and_dir(room_name, coord, dir);
        assert_eq!(pos, returned_pos);
    }
}


