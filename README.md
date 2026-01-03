
<img align="right" width="26%" src="./misc/logo.png">

Farmtrax
===

A C++ header-only library for generating AB-lines and optimizing field traversal paths for agricultural robots and tractors.

---

## Overview

Farmtrax provides sophisticated algorithms for:
- **Field Processing**: Convert GeoJSON polygons into workable farm fields with AB-lines (work paths)
- **Spatial Partitioning**: Divide fields among multiple machines for efficient parallel work
- **Path Optimization**: Generate optimal traversal sequences using graph theory and spatial indexing
- **Agricultural Patterns**: Respect real-world farming practices and field geometry

## How Farmtrax Works

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                           FARMTRAX PROCESSING PIPELINE                             │
└─────────────────────────────────────────────────────────────────────────────────────┘

1. INPUT: Field Polygon Points
   ┌─────────────────────┐
   │    GeoJSON Field    │ ──┐
   │   Boundary Points   │   │
   └─────────────────────┘   │
                             │
2. FIELD PARTITIONING         │
   ┌─────────────────────┐   │
   │   Partitioner       │◄──┘
   │ • Complex shapes    │
   │ • Multi-part split  │ ──┐
   │ • Area threshold    │   │
   └─────────────────────┘   │
                             │
3. HEADLAND GENERATION        │
   ┌─────────────────────┐   │
   │  generate_headlands │◄──┘
   │ • Buffer inward     │
   │ • Multiple rings    │ ──┐
   │ • Turn areas        │   │
   └─────────────────────┘   │
                             │
4. SWATH GENERATION           │
   ┌─────────────────────┐   │
   │  generate_swaths    │◄──┘
   │ • Parallel lines    │
   │ • AB-line creation  │ ──┐
   │ • Optimal spacing   │   │
   │ • Angle optimization│   │
   └─────────────────────┘   │
                             │
5. SPATIAL INDEXING           │
   ┌─────────────────────┐   │
   │     R-tree Build    │◄──┘
   │ • Swath endpoints   │
   │ • Bounding boxes    │ ──┐
   │ • O(log n) queries  │   │
   └─────────────────────┘   │
                             │
6. MACHINE DIVISION           │
   ┌─────────────────────┐   │
   │       Divy          │◄──┘
   │ • ALTERNATE         │
   │ • SPATIAL_RTREE     │ ──┐
   │ • LENGTH_BALANCED   │   │
   └─────────────────────┘   │
                             │
7. OBSTACLE AVOIDANCE         │
   ┌─────────────────────┐   │
   │  ObstacleAvoider    │◄──┘
   │ • Polygon inflation │
   │ • Swath cutting     │ ──┐
   │ • Connection paths  │   │
   └─────────────────────┘   │
                             │
8. GRAPH CONSTRUCTION         │
   ┌─────────────────────┐   │
   │    Boost Graph      │◄──┘
   │ • Vertex: endpoints │
   │ • Edges: work/move  │ ──┐
   │ • Weight: distance  │   │
   └─────────────────────┘   │
                             │
9. PATH OPTIMIZATION          │
   ┌─────────────────────┐   │
   │       Nety          │◄──┘
   │ • Field traversal   │
   │ • Pattern scoring   │ ──┐
   │ • Spatial clusters  │   │
   │ • Direction aware   │   │
   └─────────────────────┘   │
                             │
10. OUTPUT: Optimized Path    │
    ┌────────────────────┐   │
    │  Ordered Swaths    │◄──┘
    │ • Work sequences   │
    │ • Connection moves │
    │ • Minimal distance │
    └────────────────────┘

    Data Flow Detail:

    Field Polygon → [Partitioner] → Parts[]
                              ↓
    Parts[] → [Headland Gen] → Parts[].headlands[]
                              ↓
    Parts[] → [Swath Gen] → Parts[].swaths[] (AB-lines)
                              ↓
    Swaths[] → [R-tree Build] → Spatial Index
                              ↓
    Swaths[] → [Divy] → Machine Assignment
                              ↓
    Machine Swaths[] → [ObstacleAvoider] → Cut/Connected Swaths[]
                              ↓
    Swaths[] → [Nety Graph] → Graph(Vertices, Edges)
                              ↓
    Graph + Start Point → [Traversal Algorithm] → Optimized Path[]
```

## Key Features

- 🚜 **Agricultural AB-Line Generation**: Create work paths with A and B endpoints
- 🗺️ **Multi-Machine Division**: Intelligent field partitioning for fleet operations  
- 🌾 **Field-Aware Traversal**: Graph-based path optimization with spatial clustering
- 📊 **R-tree Spatial Indexing**: Efficient nearest-neighbor queries for large fields
- 📈 **Real-time Visualization**: Integration with Rerun for 3D field visualization
- ⚡ **High Performance**: Optimized algorithms for real-time agricultural applications
- 🛡️ **Obstacle Avoidance**: Dynamic path cutting and reconnection around obstacles
- 🎯 **Pattern Recognition**: Agricultural-aware path patterns and field geometry respect

## Processing Steps Explained

### 1. **Field Input & Partitioning**
- Load GeoJSON polygon boundaries
- **Intelligent Multi-Criteria Field Partitioning**: Advanced polygon splitting system
  - **Area-based splitting**: Split fields exceeding size thresholds (default 5 hectares)
  - **Bridge detection**: Identify and split narrow connections using morphological erosion
  - **Convexity analysis**: Split highly concave fields (below 60% convexity ratio)
  - **Aspect ratio control**: Split elongated fields with excessive length/width ratios
  - **Tooth/extension detection**: Identify and separate field protrusions and concavities
  - **Recursive partitioning**: Apply multiple criteria with configurable depth limits
- Handle irregular shapes and multiple disconnected areas
- Configurable partitioning criteria for different agricultural scenarios

### 2. **Headland Generation**
- Create turning areas by buffering field boundaries inward
- Generate multiple headland rings for different vehicle sizes
- Ensure proper turning radius for agricultural machinery

### 3. **Swath (AB-line) Generation**
- Create parallel work lines across the interior field area
- Optimize line angle for minimal number of passes
- Generate A and B endpoints for each work line
- Respect field geometry and minimize short segments

### 4. **Spatial Indexing with R-trees**
- Build efficient spatial data structures for fast queries
- Index swath endpoints, bounding boxes, and field boundaries
- Enable O(log n) nearest-neighbor searches for large fields

### 5. **Machine Division Strategies**
- **ALTERNATE**: Distribute swaths in alternating pattern for load balancing
- **SPATIAL_RTREE**: Use spatial proximity for efficient machine routing
- **LENGTH_BALANCED**: Balance workload by total swath length per machine

### 6. **Obstacle Avoidance**
- Inflate obstacle polygons by machine safety margin
- Cut swaths that intersect obstacles into smaller segments
- Create connection paths between cut segments
- Maintain work continuity while avoiding hazards

### 7. **Graph Construction**
- Build Boost Graph with vertices at each AB-line endpoint
- Create edges for both work paths (A→B) and transitions (between lines)
- Weight edges by actual distances and movement costs

### 8. **Path Optimization (Nety Algorithm)**
- **Progressive Spatial Search**: Start with 15m radius, expand to 500m
- **Agricultural Pattern Scoring**: Prefer parallel line clusters
- **Direction-Aware Connections**: Minimize cross-field movements
- **Field Pattern Recognition**: Maintain consistent farming patterns

### 9. **Output Generation**
- Reorder swaths according to optimal traversal sequence
- Generate connection moves between work areas
- Provide complete path with distance metrics and timing

---

## Installation

### CMake

```cmake
FetchContent_Declare(
  farmtrax
  GIT_REPOSITORY https://github.com/bresilla/farmtrax.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(farmtrax)

target_link_libraries(<lib/bin> PRIVATE farmtrax::farmtrax)
```

### Dependencies

- **Boost Geometry**: For spatial operations and R-tree indexing
- **Boost Graph**: For graph algorithms (Dijkstra, traversal)
- **Concord**: Coordinate system transformations
- **GeoSON**: GeoJSON parsing and writing
- **Rerun** (optional): For 3D visualization

---

## Quick Start

### Basic Usage

```cpp
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp"
#include "farmtrax/divy.hpp"
#include "farmtrax/avoid.hpp"

// 1. Load field from GeoJSON
concord::Polygon poly;
auto fc = geoson::ReadFeatureCollection("field.geojson");
poly = std::get<concord::Polygon>(fc.features[0].geometry);

// 2. Create field with headlands and AB-lines
concord::Datum datum{51.989, 5.658, 53.801};
farmtrax::Field field(poly, 0.1, datum, true, 0.5);
field.gen_field(4.0, 0.0, 3); // 4m spacing, 0° angle, 3 headland rings

// 3. Divide field among machines
auto fieldPtr = std::make_shared<farmtrax::Field>(field);
farmtrax::Divy divy(fieldPtr, farmtrax::DivisionType::ALTERNATE, 4);

// 4. Set up obstacle avoidance (optional)
std::vector<concord::Polygon> obstacles = {/* obstacle polygons */};
farmtrax::ObstacleAvoider avoider(obstacles, datum);

// 5. Generate optimal paths for each machine
auto& res = divy.result();
for (size_t m = 0; m < 4; ++m) {
    if (res.swaths_per_machine.at(m).empty()) continue;
    
    // Apply obstacle avoidance with 2.0m inflation distance
    auto avoided_swaths = avoider.avoid(res.swaths_per_machine.at(m), 2.0f);
    
    // Create graph-based path optimizer (filters to only work swaths)
    farmtrax::Nety nety(avoided_swaths);
    
    // Generate optimal traversal path
    nety.field_traversal(); // Reorders swaths internally
    
    std::cout << "Machine " << m << " optimized path: " 
              << nety.get_swaths().size() << " work segments\n";
}
```

### Working with Individual Steps

```cpp
// Step-by-step processing for detailed control

// 1. Field partitioning
farmtrax::Field field(polygon, 0.1, datum);
std::cout << "Field split into " << field.get_parts().size() << " parts\n";

// 2. Generate headlands first
field.gen_field(4.0, 0.0, 3);
for (const auto& part : field.get_parts()) {
    std::cout << "Part has " << part.headlands.size() << " headland rings\n";
    std::cout << "Part has " << part.swaths.size() << " work swaths\n";
}

// 3. Spatial indexing (automatic)
// R-trees are built automatically for efficient queries

// 4. Machine division with different strategies
farmtrax::Divy divy_alternate(fieldPtr, farmtrax::DivisionType::ALTERNATE, 4);
farmtrax::Divy divy_spatial(fieldPtr, farmtrax::DivisionType::SPATIAL_RTREE, 4);
farmtrax::Divy divy_balanced(fieldPtr, farmtrax::DivisionType::LENGTH_BALANCED, 4);

// 5. Obstacle avoidance processing
auto machine_swaths = divy_alternate.result().swaths_per_machine.at(0);
auto safe_swaths = avoider.avoid(machine_swaths, 2.0f);

// 6. Graph construction and optimization
farmtrax::Nety optimizer(safe_swaths);
optimizer.field_traversal(); // Creates optimal traversal order

// 7. Access results
const auto& optimized_swaths = optimizer.get_swaths();
for (size_t i = 0; i < optimized_swaths.size(); ++i) {
    const auto& swath = optimized_swaths[i];
    std::cout << "Segment " << i << ": " 
              << (swath->type == farmtrax::SwathType::Swath ? "WORK" : "MOVE")
              << " from (" << swath->getHead().enu.x << "," << swath->getHead().enu.y << ")"
              << " to (" << swath->getTail().enu.x << "," << swath->getTail().enu.y << ")\n";
}
```

---

## Core Components

### 1. Field Generation (`farmtrax::Field`)

Converts polygon boundaries into workable farm fields:

```cpp
farmtrax::Field field(polygon, resolution, datum, add_headlands, headland_width);
field.gen_field(spacing, angle, num_headland_rings);
```

- **Polygon Processing**: Handles complex field boundaries with holes
- **Partitioning**: Automatically splits large/complex fields into manageable parts
- **Headland Generation**: Creates turning areas by inward buffering
- **AB-Line Generation**: Creates parallel work paths across the interior field
- **Spatial Indexing**: Builds R-trees for efficient spatial queries

### 2. Machine Division (`farmtrax::Divy`)

Intelligently partitions work among multiple machines:

```cpp
farmtrax::Divy divy(field_ptr, farmtrax::DivisionType::ALTERNATE, num_machines);
```

**Division Types:**
- `ALTERNATE`: Alternating swath assignment for balanced workload
- `SPATIAL_RTREE`: Spatial proximity clustering for minimal inter-machine travel
- `LENGTH_BALANCED`: Equal total work length distribution
- `BLOCK`: Contiguous blocks for minimal inter-machine conflicts

### 3. Obstacle Avoidance (`farmtrax::ObstacleAvoider`)

Handles dynamic obstacle avoidance in field operations:

```cpp
farmtrax::ObstacleAvoider avoider(obstacles, datum);
auto safe_swaths = avoider.avoid(input_swaths, inflation_distance);
```

**Features:**
- **Polygon Inflation**: Expands obstacles by safety margin
- **Swath Cutting**: Divides work lines around obstacles
- **Connection Generation**: Creates transition paths between cut segments
- **Smart Reconnection**: Maintains work flow continuity

### 4. Path Optimization (`farmtrax::Nety`)

Advanced graph-based traversal optimization using spatial algorithms:

```cpp
farmtrax::Nety nety(swaths);
nety.field_traversal(); // Optimizes and reorders swaths
```

**Optimization Features:**
- **Field-Aware Patterns**: Respects agricultural work patterns
- **Spatial Clustering**: Groups nearby parallel lines
- **Progressive Search**: Expanding radius nearest-neighbor queries
- **Direction Optimization**: Minimizes cross-field movements

---

## Advanced Field Partitioning System

Farmtrax includes a sophisticated **multi-criteria field partitioning system** that intelligently splits complex agricultural fields into manageable parts. This system goes far beyond simple area-based splitting.

### Partitioning Strategies

The `farmtrax::Partitioner` class implements five distinct splitting strategies applied in priority order:

#### 1. **Area-Based Splitting** (Highest Priority)
```cpp
farmtrax::Partitioner::PartitionCriteria criteria;
criteria.max_area = 50000.0;  // 5 hectares maximum
```
- Splits fields exceeding area thresholds
- Uses geometric center-line cutting
- Splits along the longer dimension for optimal shapes

#### 2. **Bridge Detection** (Narrow Connection Analysis)
```cpp
criteria.min_bridge_width = 20.0;        // Minimum bridge width in meters
criteria.enable_bridge_detection = true;
```
- Uses **morphological erosion** to detect narrow connections
- Applies negative buffer operations to identify bottlenecks
- Automatically splits at narrowest connection points

#### 3. **Tooth/Extension Detection** (Concavity Analysis)
```cpp
criteria.tooth_threshold = 0.3;         // 30% area threshold for teeth
criteria.enable_tooth_detection = true;
```
- Calculates **convex hull difference** to find concave areas
- Identifies field "teeth" and protruding extensions
- Splits across significant concavities exceeding threshold

#### 4. **Aspect Ratio Control** (Elongation Splitting)
```cpp
criteria.max_aspect_ratio = 4.0;        // Maximum length/width ratio
criteria.enable_aspect_splitting = true;
```
- Prevents overly elongated field parts
- Maintains workable field proportions
- Optimizes for machinery turning patterns

#### 5. **Convexity Analysis** (Shape Quality Control)
```cpp
criteria.min_convexity = 0.6;           // Minimum 60% convexity ratio
```
- Calculates `polygon_area / convex_hull_area` ratio
- Splits highly irregular, non-convex shapes
- Ensures workable field geometry

### Advanced Configuration

```cpp
// Create partitioner with custom criteria
farmtrax::Partitioner partitioner(field_polygon, datum);

farmtrax::Partitioner::PartitionCriteria custom_criteria;
custom_criteria.max_area = 25000.0;              // 2.5 hectares max
custom_criteria.min_convexity = 0.75;            // Stricter convexity
custom_criteria.max_aspect_ratio = 3.0;          // Prevent elongation
custom_criteria.min_bridge_width = 15.0;         // Aggressive bridge detection
custom_criteria.tooth_threshold = 0.2;           // Sensitive tooth detection
custom_criteria.max_recursion_depth = 6;         // Deep recursive splitting
custom_criteria.enable_bridge_detection = true;
custom_criteria.enable_tooth_detection = true;
custom_criteria.enable_aspect_splitting = true;

// Apply intelligent partitioning
auto field_parts = partitioner.partition(25000.0, custom_criteria);
```

### Integration with Field Processing

The partitioning system is seamlessly integrated into the main `Field` class:

```cpp
// Automatic partitioning during field creation
farmtrax::Field field(polygon, 0.1, datum, true, 0.7, 15000.0); // 1.5 hectare threshold

// Each part gets independent processing
for (const auto& part : field.get_parts()) {
    // Each part has: headlands, swaths, spatial indices
    std::cout << "Part: " << part.headlands.size() << " headlands, " 
              << part.swaths.size() << " swaths\n";
}
```

### Real-World Applications

- **Complex Field Shapes**: L-shaped, T-shaped, and irregular boundaries
- **Bridge Fields**: Fields connected by narrow access routes
- **Concave Fields**: Fields with internal concavities or "bites"
- **Large Fields**: Automatically split oversized fields for efficient management
- **Multi-Machine Operations**: Create optimal work zones for multiple machines

---

## Advanced Field Partitioning System

Farmtrax features a sophisticated **multi-criteria field partitioning system** that intelligently splits complex agricultural fields into manageable work areas. The system goes far beyond simple area-based splitting to handle real-world field complexities.

### Multi-Criteria Partitioner (`farmtrax::Partitioner`)

The intelligent partitioner uses **five different splitting strategies** in priority order:

#### 1. **Area-Based Splitting** (Primary Constraint)
```cpp
criteria.max_area = 50000.0;            // Maximum area in square meters (5 hectares)
```
- Ensures field parts remain within manageable size limits
- Splits along the **longer dimension** for optimal machinery access
- Foundation for all other splitting criteria

#### 2. **Narrow Bridge Detection** (Connectivity Analysis)
```cpp
criteria.min_bridge_width = 20.0;       // Minimum bridge width in meters
criteria.enable_bridge_detection = true;
```
- Uses **polygon erosion** techniques to detect narrow connections
- Identifies bottlenecks that create inefficient machinery paths
- Automatically splits at the narrowest connection points

#### 3. **Tooth/Extension Detection** (Concavity Analysis)
```cpp
criteria.tooth_threshold = 0.3;         // 30% area threshold for teeth
criteria.enable_tooth_detection = true;
```
- Calculates **convex hull difference** to find concave areas
- Identifies field "teeth" and protruding extensions
- Splits across significant concavities exceeding threshold

#### 4. **Aspect Ratio Control** (Elongation Splitting)
```cpp
criteria.max_aspect_ratio = 4.0;        // Maximum length/width ratio
criteria.enable_aspect_splitting = true;
```
- Prevents overly elongated field parts
- Maintains workable field proportions
- Optimizes for machinery turning patterns

#### 5. **Convexity Analysis** (Shape Quality Control)
```cpp
criteria.min_convexity = 0.6;           // Minimum 60% convexity ratio
```
- Measures polygon **area/convex_hull_area ratio**
- Ensures field parts have reasonable geometric properties
- Fallback splitting when other criteria fail

### Advanced Configuration

```cpp
// Create partitioner with intelligent defaults
farmtrax::Partitioner partitioner(field_polygon, datum);

// Configure custom criteria for specific agricultural needs
farmtrax::Partitioner::PartitionCriteria criteria;
criteria.max_area = 15000.0;                    // 1.5 hectares max
criteria.min_convexity = 0.7;                   // 70% convexity required
criteria.max_aspect_ratio = 3.0;                // 3:1 max length/width
criteria.min_bridge_width = 15.0;               // 15m minimum bridge width
criteria.tooth_threshold = 0.25;                // 25% area threshold
criteria.max_recursion_depth = 6;               // Deep recursive splitting
criteria.enable_bridge_detection = true;
criteria.enable_tooth_detection = true;
criteria.enable_aspect_splitting = true;

// Apply intelligent partitioning
auto field_parts = partitioner.partition(15000.0, criteria);
std::cout << "Field intelligently split into " << field_parts.size() << " parts\n";
```

### Geometric Algorithms

The partitioner employs sophisticated **Boost Geometry algorithms**:

- **Buffer Operations**: Erosion-based bridge detection
- **Convex Hull Analysis**: Shape complexity measurement
- **Boolean Operations**: Polygon cutting and splitting
- **Centroid Calculations**: Optimal split line placement
- **Envelope Analysis**: Bounding box-based aspect ratios

### Integration with Processing Pipeline

```cpp
// Automatic integration during field creation
farmtrax::Field field(polygon, resolution, datum, centered, overlap, area_threshold);
// Field is automatically partitioned based on area_threshold

// Access partitioned field parts
for (const auto& part : field.get_parts()) {
    std::cout << "Part: " << part.headlands.size() << " headlands, " 
              << part.swaths.size() << " swaths\n";
}
```

### Agricultural Benefits

- **Machinery Efficiency**: Eliminates long traverses between disconnected areas
- **Pattern Recognition**: Identifies field geometry patterns for optimal work sequences
- **Obstacle Integration**: Works seamlessly with obstacle avoidance systems
- **Scalability**: Handles fields from small plots to large commercial operations
- **Quality Assurance**: Ensures each part is workable by agricultural machinery

---

## Graph + R-tree Architecture

The core innovation of Farmtrax is the **Nety** class, which combines graph theory with spatial indexing for optimal field traversal.

### Architecture Overview

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   AB Lines      │───▶│   Nety Class     │───▶│ Optimal Path    │
│ (Work Paths)    │    │                  │    │ (Vertex List)   │
└─────────────────┘    │ ┌──────────────┐ │    └─────────────────┘
                       │ │  R-tree      │ │
                       │ │  Spatial     │ │
                       │ │  Index       │ │
                       │ └──────────────┘ │
                       │ ┌──────────────┐ │
                       │ │  Boost       │ │
                       │ │  Graph       │ │
                       │ └──────────────┘ │
                       └──────────────────┘
```

### R-tree Spatial Indexing

The R-tree provides **O(log n)** spatial queries for efficient nearest-neighbor searches:

```cpp
using EndpointTree = boost::geometry::index::rtree<PointRTreeValue, 
                     boost::geometry::index::quadratic<16>>;
```

**Key Benefits:**
- **Fast Spatial Queries**: Find nearby endpoints in logarithmic time
- **Progressive Search**: Expanding radius search (15m → 30m → 50m → 100m → 200m → 500m)
- **Spatial Clustering**: Groups nearby parallel lines to minimize cross-field jumps

### Graph Structure

Each AB line creates two vertices (A and B endpoints) connected by edges:

```cpp
// Graph components
using Graph = boost::adjacency_list<boost::listS, boost::vecS, 
                                   boost::undirectedS, VertexProps, EdgeProps>;
std::vector<Vertex> vertex_A_; // A endpoints for each line  
std::vector<Vertex> vertex_B_; // B endpoints for each line
```

**Edge Types:**
1. **Work Edges**: A↔B connections (actual field work)
2. **Transition Edges**: Connections between different AB lines (movement)

### Field-Aware Traversal Algorithm

The traversal algorithm combines multiple heuristics:

#### 1. Progressive Spatial Search
```cpp
const std::vector<double> search_radii = {15.0, 30.0, 50.0, 100.0, 200.0, 500.0};
```
- Starts with tight 15m radius for local clustering
- Expands progressively if no suitable candidates found
- Prevents excessive cross-field connections

#### 2. Agricultural Pattern Scoring
```cpp
double score = distance + direction_penalty - pattern_bonus;
```

**Distance Component**: Base cost (shorter is better)
**Direction Penalty**: Prefers perpendicular movement to line orientation
**Pattern Bonus**: Rewards maintaining parallel line clusters

#### 3. Orientation-Aware Connections
```cpp
bool lines_have_similar_orientation(size_t line1_id, size_t line2_id) const {
    // Check if lines are parallel within 15° tolerance
    return angle < (15.0 * M_PI / 180.0);
}
```

### Connection Quality Control

Smart connection management prevents unrealistic paths:

- **Distance Threshold**: Maximum 150m connection radius
- **Connection Limit**: At most 4 connections per endpoint  
- **Parallel Preference**: 50% penalty for non-parallel connections
- **Clustering Bonus**: Rewards local field patterns

### Performance Characteristics

| Operation | Time Complexity | Description |
|-----------|----------------|-------------|
| R-tree Insert | O(log n) | Adding endpoints to spatial index |
| Nearest Neighbor | O(log n) | Finding closest unvisited endpoints |
| Graph Construction | O(n log n) | Building connectivity graph |
| Dijkstra Path | O((V + E) log V) | Shortest path calculation |
| Field Traversal | O(n² log n) | Complete field traversal |

### Real-World Benefits

1. **Reduced Travel Time**: Minimizes non-productive movement between work areas
2. **Fuel Efficiency**: Shorter transition paths reduce fuel consumption  
3. **Field Preservation**: Avoids excessive cross-field traffic
4. **Scalable Performance**: Handles large fields with hundreds of AB lines
5. **Agricultural Compliance**: Respects real farming practices and patterns

---

## Examples

### Complete Field Processing Pipeline

```cpp
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp" 
#include "farmtrax/divy.hpp"
#include "farmtrax/avoid.hpp"
#include "farmtrax/utils/visualize.hpp"

int main() {
    // Load and process field
    auto fc = geoson::ReadFeatureCollection("field.geojson");
    concord::Polygon poly = std::get<concord::Polygon>(fc.features[0].geometry);
    
    concord::Datum datum{51.989, 5.658, 53.801};
    farmtrax::Field field(poly, 0.1, datum, true, 0.5);
    field.gen_field(4.0, 0.0, 3); // 4m spacing, auto angle, 3 headland rings
    
    // Load obstacles (trees, buildings, etc.)
    std::vector<concord::Polygon> obstacles;
    // ... load obstacle polygons from GeoJSON or other sources
    farmtrax::ObstacleAvoider avoider(obstacles, datum);
    
    // Multi-machine division
    auto fieldPtr = std::make_shared<farmtrax::Field>(field);
    farmtrax::Divy divy(fieldPtr, farmtrax::DivisionType::SPATIAL_RTREE, 4);
    
    // Set up visualization (optional)
    auto rec = std::make_shared<rerun::RecordingStream>("farmtrax", "space");
    rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy");
    farmtrax::visualize::show_field(field, rec);
    
    // Process each machine
    auto& res = divy.result();
    for (size_t m = 0; m < 4; ++m) {
        if (res.swaths_per_machine.at(m).empty()) continue;
        
        std::cout << "Machine " << m << " original swaths: " 
                  << res.swaths_per_machine.at(m).size() << "\n";
        
        // Apply obstacle avoidance with 2.0 meter inflation distance
        auto avoided_swaths = avoider.avoid(res.swaths_per_machine.at(m), 2.0f);
        std::cout << "After obstacle avoidance: " << avoided_swaths.size() << " segments\n";
        
        // Create and optimize path (filters to only work swaths)
        farmtrax::Nety nety(avoided_swaths);
        nety.field_traversal(); // Reorders swaths for optimal path
        
        // Calculate performance metrics
        const auto& optimized_swaths = nety.get_swaths();
        double total_work = 0.0, total_move = 0.0;
        
        for (const auto& swath : optimized_swaths) {
            double length = swath->line.length();
            if (swath->type == farmtrax::SwathType::Swath) {
                total_work += length;
            } else {
                total_move += length;
            }
        }
        
        std::cout << "Machine " << m << " final path: " << optimized_swaths.size() 
                  << " segments (" << total_work << "m work, " << total_move << "m movement)\n";
        
        // Visualize results
        farmtrax::visualize::show_avoided_swaths(avoided_swaths, rec, m, "avoided");
        farmtrax::visualize::show_swath_tour(nety, rec, m);
    }
    
    return 0;
}
```

### Custom Obstacle Handling

```cpp
// Define custom obstacles
std::vector<concord::Polygon> obstacles;

// Add a circular obstacle (tree)
concord::Polygon tree;
int num_points = 16;
double radius = 5.0; // 5 meter radius
double center_x = 100.0, center_y = 200.0;
for (int i = 0; i < num_points; ++i) {
    double angle = 2.0 * M_PI * i / num_points;
    concord::Point pt;
    pt.enu.x = center_x + radius * cos(angle);
    pt.enu.y = center_y + radius * sin(angle);
    tree.addPoint(pt);
}
tree.addPoint(tree.getPoints().front()); // Close polygon
obstacles.push_back(tree);

// Add a rectangular obstacle (building)
concord::Polygon building;
building.addPoint(concord::Point{concord::ENU{50, 150, 0}});
building.addPoint(concord::Point{concord::ENU{70, 150, 0}});
building.addPoint(concord::Point{concord::ENU{70, 180, 0}});
building.addPoint(concord::Point{concord::ENU{50, 180, 0}});
building.addPoint(building.getPoints().front());
obstacles.push_back(building);

// Create avoider with 3.0m safety margin
farmtrax::ObstacleAvoider avoider(obstacles, datum);
auto safe_swaths = avoider.avoid(original_swaths, 3.0f);

// Inspect results
for (const auto& swath : safe_swaths) {
    if (swath->type == farmtrax::SwathType::Swath) {
        std::cout << "Work segment: " << swath->line.length() << "m\n";
    } else if (swath->type == farmtrax::SwathType::Around) {
        std::cout << "Avoidance connection: " << swath->line.length() << "m\n";
    }
}
```

### Advanced Path Optimization

```cpp
// Create optimizer with multiple start point strategies
farmtrax::Nety nety(swaths);

// Strategy 1: Field traversal from automatic start point
nety.field_traversal();
auto path1 = nety.get_swaths();

// Strategy 2: Shortest path between specific points
farmtrax::BPoint custom_start(field_boundary.min_x, field_boundary.min_y);
farmtrax::BPoint custom_end(field_boundary.max_x, field_boundary.max_y);
nety.shortest_path(custom_start, custom_end);
auto path2 = nety.get_swaths();

// Compare strategies
double distance1 = 0.0, distance2 = 0.0;
for (const auto& swath : path1) {
    distance1 += swath->line.length();
}
for (const auto& swath : path2) {
    distance2 += swath->line.length();
}

std::cout << "Field traversal: " << distance1 << "m total\n";
std::cout << "Shortest path: " << distance2 << "m total\n";
std::cout << "Best strategy: " << (distance1 < distance2 ? "Field traversal" : "Shortest path") << "\n";
```

---

## Build Instructions

### Prerequisites

```bash
# Ubuntu/Debian
sudo apt install cmake build-essential libboost-all-dev

# macOS  
brew install cmake boost

# Or use devbox (recommended)
devbox shell
```

### Building

```bash
# Using provided build script
./run.sh b    # Build
./run.sh r    # Run example

# Manual CMake
mkdir build && cd build
cmake ..
make -j$(nproc)
./main
```

---

## Visualization

Farmtrax integrates with [Rerun](https://rerun.io/) for real-time 3D visualization:

```cpp
auto rec = std::make_shared<rerun::RecordingStream>("farmtrax", "space");
rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy");

farmtrax::visualize::show_field(field, rec);
farmtrax::visualize::show_swath_tour(nety, path, rec, machine_id);
```

**Visualization Features:**
- 3D field boundaries and topography
- AB line visualization with directional arrows
- Machine path visualization with color coding
- Real-time path optimization updates
- Connection quality indicators

---

## Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---
