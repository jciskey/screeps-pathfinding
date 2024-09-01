
use std::collections::HashMap;
use std::hash::Hash;

use serde::{Serialize, Deserialize};

use screeps::{Position};

/// A simple cache for path data.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PathCache<K>
where
    K: PartialEq + Eq + Hash
{
    cache: HashMap<K, Vec<Position>>
}

impl<K> PathCache<K>
where
    K: PartialEq + Eq + Hash + Clone
{
    /// Initializes a new, empty terrain cache.
    pub fn new() -> Self {
        Self {
            cache: HashMap::new(),
        }
    }

    /// Returns the cached path, if it exists. Returns None if the path isn't cached.
    ///
    /// # Examples
    /// ```rust
    /// use screeps::{Direction, Position, RoomCoordinate};
    /// use screeps_pathfinding::utils::cache::PathCache;
    /// 
    /// // Create a new path cache that uses String as the key type
    /// let mut cache: PathCache<String> = PathCache::new();
    /// 
    /// // Build a path
    /// let start = Position::new(
    ///     RoomCoordinate::try_from(24).unwrap(),
    ///     RoomCoordinate::try_from(18).unwrap(),
    ///     "E5N6".parse().unwrap(),
    /// );
    /// let goal = start.checked_add_direction(Direction::Right).unwrap();
    /// let path = vec!(start, goal);
    /// 
    /// let existing_key = "existing_key".to_string();
    /// 
    /// // Actually store the path in the cache
    /// assert_eq!(cache.is_path_cached(&existing_key), false);
    /// cache.update_cached_path(&existing_key, path.into_iter());
    /// assert_eq!(cache.is_path_cached(&existing_key), true);
    /// 
    /// // Pull the cached copy of the path
    /// let path_opt = cache.get_cached_path(&existing_key);
    /// assert!(path_opt.is_some());
    /// 
    /// let path_slice = path_opt.unwrap();
    /// assert!(path_slice.len() == 2);
    /// assert!(path_slice[0] == start);
    /// assert!(path_slice[1] == goal);
    /// 
    /// // Attempt to pull a non-existent path entry
    /// let nonexisting_key = "nonexisting_key".to_string();
    /// let path_opt = cache.get_cached_path(&nonexisting_key);
    /// assert!(path_opt.is_none());
    /// assert_eq!(cache.is_path_cached(&nonexisting_key), false);
    /// ```
    pub fn get_cached_path(&self, path_key: &K) -> Option<&[Position]> {
        self.cache.get(path_key).map(|v| &**v)
    }

    /// Returns whether a path is cached with the provided path key.
    ///
    /// # Examples
    /// ```rust
    /// use screeps::{Direction, Position, RoomCoordinate};
    /// use screeps_pathfinding::utils::cache::PathCache;
    /// 
    /// // Create a new path cache that uses String as the key type
    /// let mut cache: PathCache<String> = PathCache::new();
    /// 
    /// // A path doesn't exist yet in the cache for this key
    /// let key = "some_key".to_string();
    /// assert_eq!(cache.is_path_cached(&key), false);
    /// 
    /// // Build a path
    /// let start = Position::new(
    ///     RoomCoordinate::try_from(24).unwrap(),
    ///     RoomCoordinate::try_from(18).unwrap(),
    ///     "E5N6".parse().unwrap(),
    /// );
    /// let goal = start.checked_add_direction(Direction::Right).unwrap();
    /// let path = vec!(start, goal);
    /// 
    /// // Store the path in the cache
    /// cache.update_cached_path(&key, path.into_iter());
    /// 
    /// assert_eq!(cache.is_path_cached(&key), true);
    /// ```
    pub fn is_path_cached(&self, path_key: &K) -> bool {
        self.cache.contains_key(path_key)
    }

    /// Returns the path, generating and caching it if it's not already cached.
    ///
    /// Returns None only if the path is not cached and the generation function returns None.
    ///
    /// # Examples
    /// ```rust
    /// use screeps::{Direction, Position, RoomCoordinate};
    /// use screeps_pathfinding::utils::cache::PathCache;
    /// 
    /// // Create a new path cache that uses String as the key type
    /// let mut cache: PathCache<String> = PathCache::new();
    /// 
    /// // The path doesn't exist yet in the cache
    /// let key = "some_key".to_string();
    /// assert_eq!(cache.is_path_cached(&key), false);
    /// 
    /// let start = Position::new(
    ///     RoomCoordinate::try_from(24).unwrap(),
    ///     RoomCoordinate::try_from(18).unwrap(),
    ///     "E5N6".parse().unwrap(),
    /// );
    /// let goal = start.checked_add_direction(Direction::Right).unwrap();
    /// 
    /// // Pull the path, generating it since it doesn't exist
    /// let path_opt = cache.get_path(&key, |path_key| {
    ///     // Build a path
    ///     let path = vec!(start, goal);
    ///     Some(path)
    /// });
    /// assert!(path_opt.is_some());
    /// 
    /// let path_slice = path_opt.unwrap();
    /// assert!(path_slice.len() == 2);
    /// assert!(path_slice[0] == start);
    /// assert!(path_slice[1] == goal);
    /// 
    /// assert_eq!(cache.is_path_cached(&key), true);
    /// ```
    pub fn get_path<G>(&mut self, path_key: &K, generator_fn: G) -> Option<&[Position]>
    where
        G: FnOnce(&K) -> Option<Vec<Position>>,
    {
        if !self.is_path_cached(path_key) {
            // We don't have a cached copy of the path, generate and cache it
            let path_opt = generator_fn(path_key);
            if let Some(path) = path_opt {
                let _ = self.cache.insert(path_key.clone(), path);
            }
            else {
                return None;
            }
        }

        self.get_cached_path(path_key)
    }

    /// Updates the path cache for a specific key.
    ///
    /// This allows for pre-loading the cache with any existing
    /// path data you might already have available.
    ///
    /// # Examples
    /// ```rust
    /// use screeps::{Direction, Position, RoomCoordinate};
    /// use screeps_pathfinding::utils::cache::PathCache;
    /// 
    /// // Create a new path cache that uses String as the key type
    /// let mut cache: PathCache<String> = PathCache::new();
    /// 
    /// // Build a path
    /// let start = Position::new(
    ///     RoomCoordinate::try_from(24).unwrap(),
    ///     RoomCoordinate::try_from(18).unwrap(),
    ///     "E5N6".parse().unwrap(),
    /// );
    /// let goal = start.checked_add_direction(Direction::Right).unwrap();
    /// let path = vec!(start, goal);
    /// 
    /// let existing_key = "existing_key".to_string();
    /// 
    /// // Store the path in the cache
    /// cache.update_cached_path(&existing_key, path.into_iter());
    /// 
    /// // Pull the cached copy of the path
    /// let path_opt = cache.get_cached_path(&existing_key);
    /// assert!(path_opt.is_some());
    /// 
    /// let path_slice = path_opt.unwrap();
    /// assert!(path_slice.len() == 2);
    /// assert!(path_slice[0] == start);
    /// assert!(path_slice[1] == goal);
    /// ```
    pub fn update_cached_path(&mut self, path_key: &K, path_iter: impl Iterator<Item=Position>) {
        let path: Vec<Position> = path_iter.collect();
        let _ = self.cache.insert(path_key.clone(), path);
    }

    /// Removes the path cached for a specific key.
    pub fn remove_cached_path(&mut self, path_key: &K) {
        self.cache.remove(path_key);
    }

}
