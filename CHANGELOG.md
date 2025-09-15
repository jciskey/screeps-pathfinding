# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

## [v0.1.4]

### Added

- Added better traits for A\* to allow users to bring their own types instead of being locked into u32 as their costs.

### Changed

- Split A\* implementation into two different functions, to allow for different optimizations.

### Fixed

- Bugfix for A\* edge case where it didn't calculate the actual shortest path when using an aggressive heuristic sorting. ([Issue #4](https://github.com/jciskey/screeps-pathfinding/issues/4))

## [v0.1.3]

Updated main Screeps crate dependency to v0.23.

## [v0.1.2]

### Added

- Added caching structures for terrain (`TerrainCache`), cost matrices (`LCMCache`), and paths (`PathCache`).
- Added utility helper methods for using terrain and cost matrix caches in pathfinding algorithms.

## [v0.1.1]

### Added

- Added Dijkstra's Algorithm
- Added A\* Algorithm
- Added utility helper methods for heuristics and goals
