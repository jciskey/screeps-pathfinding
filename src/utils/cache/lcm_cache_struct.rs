
use std::collections::HashMap;

use screeps::{LocalCostMatrix, RoomName};

/// A simple passthrough cache that preferentially returns cached
/// LCM data for a room, dynamically generating LCM data as-needed
/// from a user-provided closure and caching it before returning it.
#[derive(Debug, Clone)]
pub struct LCMCache {
    cache: HashMap<RoomName, LocalCostMatrix>,
}

impl LCMCache {
    /// Initializes a new, empty LCM cache.
    pub fn new() -> Self {
        Self {
            cache: HashMap::new(),
        }
    }

    /// Returns the cached LCM, if it exists.
    ///
    /// Returns None if the LCM isn't cached.
    pub fn get_cached_lcm(&self, room_name: &RoomName) -> Option<&LocalCostMatrix> {
        self.cache.get(room_name)
    }

    /// Returns whether an LCM is cached for a particular room.
    pub fn is_lcm_cached(&self, room_name: &RoomName) -> bool {
        self.cache.contains_key(room_name)
    }

    /// Returns the room LCM, generating and caching it if it's not already cached.
    pub fn get_lcm(&mut self, room_name: &RoomName, generator_fn: impl FnOnce(&RoomName) -> LocalCostMatrix) -> &LocalCostMatrix {
        if self.cache.get(room_name).is_none() {
            // We don't have a cached copy of the LCM, generate and cache it
            let lcm = generator_fn(room_name);
            let _ = self.cache.insert(*room_name, lcm);
        }

        self.get_cached_lcm(room_name).unwrap() // We know this is okay, because we generated and inserted a new LCM for this entry
    }

    /// Updates the LCM cache for a specific room.
    ///
    /// This allows for pre-loading the cache with any existing
    /// room LCM data you might already have available.
    pub fn update_cached_lcm(&mut self, room_name: RoomName, lcm: LocalCostMatrix) {
        let _ = self.cache.insert(room_name, lcm);
    }

}
