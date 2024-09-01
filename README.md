# Screeps Pathfinding

![Crates.io Version](https://img.shields.io/crates/v/screeps-pathfinding)
![Crates.io License](https://img.shields.io/crates/l/screeps-pathfinding)

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

For convenience in actually using the library, it also includes utility functions
for common pathfinding use-cases, as well as caching structures for terrain,
cost matrices, and paths. The intent is not to provide a one-size-fits-all drop-in
navigation replacement, but instead to provide flexible building blocks that can
be used to build the appropriate solution for an individual Screeps bot.

## Current Algorithms

- Dijkstra's Shortest Path
- A\*

## Simple Timing Comparisons

Tests were done on Shard3 of MMO. Iterations were spread across multiple ticks.
Start and goal positions were static.

1 iteration:

| Algorithm | CPU Used |
|-----------|----------|
| A\*       | 0.1967   |
| Dijkstra  | 1.3069   |
| Pathfinder| 1.9285   |


5 iterations:

| Algorithm | CPU Used |
|-----------|----------|
| A\*       | 0.2142   |
| Dijkstra  | 1.3660   |
| Pathfinder| 0.6378   |


20 iterations:

| Algorithm | CPU Used |
|-----------|----------|
| A\*       | 0.2128   |
| Dijkstra  | 1.3544   |
| Pathfinder| 0.3958   |


300 iterations:

| Algorithm | CPU Used |
|-----------|----------|
| A\*       | 0.2141   |
| Dijkstra  | 1.3530   |
| Pathfinder| 0.2574   |


Of particular note, Pathfinder averages start to drop significantly after the first iteration,
which likely means that there's some JIT optimization going on in JS-Land since we're making
the exact same start-goal pathfinding calls each time.
