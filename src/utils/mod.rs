use screeps::constants::Terrain;
use screeps::local::{LocalCostMatrix, LocalRoomTerrain};

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
