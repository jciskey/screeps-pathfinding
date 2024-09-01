
use std::collections::HashMap;

use screeps::{LocalRoomTerrain, RoomName};

/// A simple passthrough cache that preferentially returns cached
/// terrain data, dynamically pulling terrain data as-needed
/// from game state and caching it before returning it.
#[derive(Debug, Clone)]
pub struct TerrainCache {
    cache: HashMap<RoomName, LocalRoomTerrain>
}

impl TerrainCache {
    /// Initializes a new, empty terrain cache.
    pub fn new() -> Self {
        Self {
            cache: HashMap::new(),
        }
    }

    /// Returns the cached room terrain, if it exists.
    ///
    /// Returns None if the room terrain isn't cached.
    pub fn get_cached_terrain(&self, room_name: &RoomName) -> Option<&LocalRoomTerrain> {
        self.cache.get(room_name)
    }

    /// Returns whether the terrain is cached for a particular room.
    pub fn is_terrain_cached(&self, room_name: &RoomName) -> bool {
        self.cache.contains_key(room_name)
    }

    /// Returns the room terrain, pulling and caching it from the game state if it's not already cached.
    ///
    /// Returns None only if [screeps::objects::RoomTerrain::new] returns None.
    ///
    /// Note: This method can, but might not, pull from the game state, which requires crossing the WASM boundary.
    pub fn get_terrain(&mut self, room_name: &RoomName) -> Option<&LocalRoomTerrain> {
        if self.cache.get(room_name).is_none() {
            // We don't have a cached copy of the terrain, pull it and cache it
            let js_terrain_opt = screeps::objects::RoomTerrain::new(*room_name);
            if let Some(js_terrain) = js_terrain_opt {
                let local_terrain = screeps::local::LocalRoomTerrain::from(js_terrain);
                let _ = self.cache.insert(*room_name, local_terrain);
            }
        }

        self.get_cached_terrain(room_name)
    }

    /// Updates the terrain cache for a specific room.
    ///
    /// This allows for pre-loading the cache with any existing
    /// room terrain you might already have available.
    pub fn update_cached_terrain(&mut self, room_name: RoomName, local_terrain: LocalRoomTerrain) {
        let _ = self.cache.insert(room_name, local_terrain);
    }

}
