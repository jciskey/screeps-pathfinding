use screeps::{LocalRoomTerrain, RoomName};
use screeps_utils::offline_map::load_shard_map_json;
use std::collections::HashMap;
use std::{env, path::PathBuf};

const TEST_MAPS_DIRNAME: &str = "test_maps";

pub fn load_all_room_terrains_from_map(map_name: &str) -> HashMap<RoomName, LocalRoomTerrain> {
    let mut ret_data = HashMap::new();

    // Load map data
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let map_path = PathBuf::from(manifest_dir)
        .join("src")
        .join("common")
        .join("data")
        .join(TEST_MAPS_DIRNAME)
        .join(map_name);
    println!("{}", map_path.display());
    let map_data = load_shard_map_json(map_path);

    // Extract terrain data from each room
    for (room_name, room_data) in map_data.rooms {
        ret_data.insert(room_name, room_data.terrain);
    }

    ret_data
}
