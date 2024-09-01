use screeps::{Direction, Position};

/// Utility function for converting a position and a path
/// into a direction for the next movement on the path.
///
/// If the position is not on the path, it will return
/// a direction that moves the creep towards the first
/// position of the path.
///
/// Returns None if the current position is the final
/// position in the path, or if the path is empty.
pub fn get_next_step_direction_in_vec_pos(
    current_pos: &Position,
    path: &[Position],
) -> Option<Direction> {
    if path.is_empty() {
        return None;
    }

    for i in 0..path.len() {
        let path_pos = path[i];
        if path_pos == *current_pos {
            // Check if the current position is the last entry in the path
            if i == (path.len() - 1) {
                // info!(target: "next_step", "Current position is last entry in path");
                return None; // No next step if we're at the last entry
            } else {
                let next_pos = path[i + 1];
                let direction = current_pos.get_direction_to(next_pos);
                // info!(target: "next_step", "Direction: {:?}", direction);
                return direction;
            }
        }
    }

    // No next step if we're not on the path at all, move towards the path start
    // info!(target: "next_step", "Current position is not in path");
    if let Some(path_pos) = path.first() {
        current_pos.get_direction_to(*path_pos)
    } else {
        // Final fallback, should be captured by the initial length check, but
        // the compiler isn't smart enough to know that.
        None
    }
}

// #[derive(Debug)]
// pub struct PathMovementCache {
//     current_position: Position,
//     path_index: usize,
//     path: &[Position],
// }

// impl PathMovementCache {
//     pub fn new(current_position: Position, path_index: usize, path: &[Position]) -> Self {
//         Self {
//             current_position,
//             path_index,
//             path,
//         }
//     }

//     pub fn did_creep_move(&self) -> bool {

//     }

// }
