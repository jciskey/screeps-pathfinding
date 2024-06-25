# Screeps Pathfinding

Pathfinding algorithms in native Rust for the programmable MMO Screeps: World.

## Purpose

The intent of this library is to have multiple pathfinding algorithms implemented
in native Rust to avoid the need for crossing the WASM boundary and doing
pathfinding in JS with Pathfinder. Calling into JS will be avoided wherever
possible and will be clearly notated when unavoidable.

The intent of having multiple pathfinding algorithms available is to allow for
using more advanced algorithms than just the Jump-Point Search that Pathfinder
implements, which would be beneficial for pathfinding with moving goals and
starting positions (such as during combat).

## Current Algorithms

None `:(`
