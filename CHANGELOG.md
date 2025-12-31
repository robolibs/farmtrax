# Changelog

## [0.0.4] - 2025-12-31

### <!-- 1 -->🐛 Bug Fixes

- Improve Rerun visualization logging paths

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Update dependencies and build configurations

## [0.0.3] - 2025-12-31

### Build

- Standardize build system configuration

## [0.0.2] - 2025-12-31

### <!-- 0 -->⛰️  Features

- Simplify field boundary definition
- Add geotagged field and swath visualization
- Migrate from Boost to datapod/graphix libraries
- Convert library to header-only

### <!-- 1 -->🐛 Bug Fixes

- Correct buffer_polygon winding order and closed polygon handling
- Restore original WGS84 field coordinates in main.cpp example
- Use absolute area threshold in buffer_polygon to prevent false collapse detection
- Use pre-converted ENU coordinates in main.cpp example
- Remove unused zoneout dependency

### Build

- Refactor build system for clarity and consistency

## [1.1.1] - 2025-12-15

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Update dev environment and build system paths

## [1.1.0] - 2025-12-15

### <!-- 0 -->⛰️  Features

- Add example for visualizing Dubins and Reeds-Shepp paths

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Update concord and clean up CMakeLists
- Introduce xmake build system for comprehensive project management

## [1.0.1] - 2025-11-01

### Build

- Refactor Boost library lookup to a general scope

## [1.0.0] - 2025-11-01

### <!-- 2 -->🚜 Refactor

- Refactor build system and reorganize source files

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Refactor build system and update dependencies
- Bump project version to 0.3.3
- Refactor build system and update dependencies

## [1.0.0] - 2025-11-01

### <!-- 2 -->🚜 Refactor

- Refactor build system and reorganize source files

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Refactor build system and update dependencies

## [0.3.3] - 2025-10-31

### <!-- 0 -->⛰️  Features

- Implement Dubins and Reeds-Shepp path planning

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Update geoson and zoneout, remove plot example

### Build

- Add nlohmann_json as an external dependency
- Refactor build configuration and local development workflow

## [0.3.2] - 2025-08-04

### <!-- 2 -->🚜 Refactor

- Refactor swath visualization to include detailed tours

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Update dependencies and refine UI layout

## [0.3.1] - 2025-07-31

### <!-- 0 -->⛰️  Features

- Add equal-area partitioning option
- Refactor Field class and update dependencies
- Replace dynamic GIS with static coordinate system

### <!-- 2 -->🚜 Refactor

- Refine field visualization and border handling

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Gitignore things

## [0.3.0] - 2025-06-15

### <!-- 0 -->⛰️  Features

- Add obstacle avoidance and visualization

### <!-- 2 -->🚜 Refactor

- Refactor examples and remove dead code

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Refactor for updated concord point structure

## [0.2.1] - 2025-06-13

### <!-- 6 -->🧪 Testing

- Refine field geometry handling and testing

## [0.2.0] - 2025-06-12

### <!-- 0 -->⛰️  Features

- Use concord's polygon partitioner
- Docs: Explain and exemplify new partitioning
- Implement advanced multi-criteria field partitioning
- Add obstacle avoidance to Rerun visualization
- Implement basic obstacle avoidance functionality
- Generate and visualize connections between swaths
- Per-indivifual parts v3
- Per-indivifual parts v2
- Improve swath traversal and reordering logic
- Return swathsh instead of vertices
- Per-indivifual parts
- Use intial point automatically (still some segfault though)
- Initial swath return support for nety
- Refactor geometry constructors and simplify usage

### <!-- 1 -->🐛 Bug Fixes

- Stupid pointer issue with intil point for graph

### <!-- 2 -->🚜 Refactor

- Refactor Swath representation and handling

### <!-- 6 -->🧪 Testing

- Improve Geometry Utility Testing and Formatting

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Fix gitignore entry for compile commands
- Ignore compile_commands.json

### Build

- Refactor build system and code style

## [0.1.2] - 2025-05-30

### <!-- 0 -->⛰️  Features

- Implement agricultural field traversal networks utilizing AB lines
- Add swath tour visualization with connections
- Optimize swath path calculation with 2-opt local search
- Feat: Introduce graph algorithm to example
- Refactor and implement new division strategies
- Update fleet visualization to use Divy object
- Feat: Add support for field division visualization
- Feat: Add machine division visualization
- Visualize field swaths in example
- Refine field generation and related plotting
- Refactor geometry processing for field generation
- Refactor border polygon and add swath count to example
- Introduce noise and geometry for field generation
- Convert to geotiff and FarmTrax data structures
- Update geojson with new coordinate data
- Add GeoJSON example code and dependencies
- Incorporate concord for codebase linking
- Init

### <!-- 1 -->🐛 Bug Fixes

- Adjust polygon handling and datum

### <!-- 2 -->🚜 Refactor

- Introduce partitioning for field division
- Introduce R-trees for spatial indexing of geometry
- Rename and refactor build scripts and aliases

### <!-- 3 -->📚 Documentation

- Document the Graph + R-tree architecture

### <!-- 4 -->⚡ Performance

- Improve tracing algorithm with focus on field patterns

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Refactor release script and visualization utilities
- Update build and add C++ example


