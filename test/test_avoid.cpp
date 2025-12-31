#include "doctest/doctest.h"
#include "farmtrax/avoid.hpp"
#include "farmtrax/field.hpp"

// Helper function to create AABB from a Segment
inline datapod::AABB segment_to_aabb(const datapod::Segment &seg) {
    return datapod::AABB{datapod::Point{std::min(seg.start.x, seg.end.x), std::min(seg.start.y, seg.end.y),
                                        std::min(seg.start.z, seg.end.z)},
                         datapod::Point{std::max(seg.start.x, seg.end.x), std::max(seg.start.y, seg.end.y),
                                        std::max(seg.start.z, seg.end.z)}};
}

// Helper function to create a simple obstacle polygon
datapod::Polygon create_obstacle(const datapod::Geo &datum = datapod::Geo{}) {
    datapod::Polygon obstacle;
    // Create a simple square obstacle at (50,25) with 10m sides
    obstacle.vertices.push_back(datapod::Point{45.0, 20.0, 0.0});
    obstacle.vertices.push_back(datapod::Point{55.0, 20.0, 0.0});
    obstacle.vertices.push_back(datapod::Point{55.0, 30.0, 0.0});
    obstacle.vertices.push_back(datapod::Point{45.0, 30.0, 0.0});
    obstacle.vertices.push_back(datapod::Point{45.0, 20.0, 0.0}); // Close the polygon
    return obstacle;
}

// Helper to create a field with obstacle
std::pair<farmtrax::Field, datapod::Polygon> create_field_with_obstacle(const datapod::Geo &datum = datapod::Geo{}) {
    // Create a rectangular field - smaller and simpler for testing
    datapod::Polygon field_poly;
    field_poly.vertices.push_back(datapod::Point{0.0, 0.0, 0.0});
    field_poly.vertices.push_back(datapod::Point{100.0, 0.0, 0.0});
    field_poly.vertices.push_back(datapod::Point{100.0, 50.0, 0.0});
    field_poly.vertices.push_back(datapod::Point{0.0, 50.0, 0.0});
    field_poly.vertices.push_back(datapod::Point{0.0, 0.0, 0.0}); // Close the polygon

    // Create obstacle
    datapod::Polygon obstacle = create_obstacle(datum);

    // Create minimal Field object
    farmtrax::Field field(field_poly, datum, false, 1000.0); // Using large area threshold to avoid partitioning

    return {field, obstacle};
}

TEST_CASE("ObstacleAvoider Construction") {
    datapod::Geo datum{51.0, 5.0, 0.0};

    // Create an obstacle
    datapod::Polygon obstacle = create_obstacle(datum);
    std::vector<datapod::Polygon> obstacles = {obstacle};

    // Create avoider
    farmtrax::ObstacleAvoider avoider(obstacles, datum);

    // No real way to test internal state, but at least we verify construction doesn't throw
}

TEST_CASE("Obstacle Avoidance with Datapod Polygons") {
    datapod::Geo datum{51.0, 5.0, 0.0};

    // Create field with obstacle
    auto [field, obstacle] = create_field_with_obstacle(datum);
    std::vector<datapod::Polygon> obstacles = {obstacle};

    // Create avoider
    farmtrax::ObstacleAvoider avoider(obstacles, datum);

    // Print obstacle info for debugging
    std::cout << "Obstacle polygon points: " << obstacle.vertices.size() << std::endl;
    std::cout << "Obstacle coords: ";
    for (const auto &pt : obstacle.vertices) {
        std::cout << "(" << pt.x << "," << pt.y << ") ";
    }
    std::cout << std::endl;

    // Create our own test swaths
    std::vector<std::shared_ptr<const farmtrax::Swath>> input_swaths;

    // Create vertical swaths that will cross the obstacle at 50,25
    for (double x = 40.0; x < 60.0; x += 5.0) { // Use a smaller range to focus on the obstacle area
        auto swath = std::make_shared<farmtrax::Swath>();
        swath->type = farmtrax::SwathType::Swath;
        swath->uuid = "swath_" + std::to_string(static_cast<int>(x));

        // Create a vertical line
        datapod::Point start(x, 0.0, 0.0);
        datapod::Point end(x, 50.0, 0.0);
        swath->line.start = start;
        swath->line.end = end;

        // Create the bounding box
        swath->bounding_box = segment_to_aabb(swath->line);

        input_swaths.push_back(swath);
    }

    std::cout << "Created " << input_swaths.size() << " test swaths for obstacle avoidance" << std::endl;

    // Apply avoidance with 2.0m inflation
    auto avoided_swaths = avoider.avoid(input_swaths, 2.0);

    // Verify avoidance results
    CHECK(!avoided_swaths.empty());

    // Should have more output swaths than input if splits occurred
    CHECK(avoided_swaths.size() >= input_swaths.size());

    // Each avoided swath should be a different pointer than input swaths
    bool found_new_swath = false;
    for (const auto &out_swath : avoided_swaths) {
        bool is_original = false;
        for (const auto &in_swath : input_swaths) {
            if (out_swath.get() == in_swath.get()) {
                is_original = true;
                break;
            }
        }
        if (!is_original) {
            found_new_swath = true;
            break;
        }
    }
    CHECK(found_new_swath);
}

TEST_CASE("Obstacle Avoidance with Different Inflation Distances") {
    datapod::Geo datum{51.0, 5.0, 0.0};

    // Create field with obstacle
    auto [field, obstacle] = create_field_with_obstacle(datum);
    std::vector<datapod::Polygon> obstacles = {obstacle};

    // Create avoider
    farmtrax::ObstacleAvoider avoider(obstacles, datum);

    // Create our own test swaths
    std::vector<std::shared_ptr<const farmtrax::Swath>> input_swaths;

    // Create vertical swaths that will cross the obstacle
    for (double x = 10.0; x < 100.0; x += 10.0) {
        auto swath_ptr = std::make_shared<farmtrax::Swath>();
        swath_ptr->type = farmtrax::SwathType::Swath;
        swath_ptr->uuid = "swath_" + std::to_string(static_cast<int>(x));

        // Create a vertical line
        datapod::Point start(x, 0.0, 0.0);
        datapod::Point end(x, 50.0, 0.0);
        swath_ptr->line.start = start;
        swath_ptr->line.end = end;

        // Ensure bounding box is computed
        swath_ptr->bounding_box = segment_to_aabb(swath_ptr->line);

        input_swaths.push_back(swath_ptr);
    }

    // Test with different inflation distances
    SUBCASE("No inflation") {
        auto avoided_swaths = avoider.avoid(input_swaths, 0.0);
        // With no inflation, the result should be similar to input
        // (might have some changes due to exact intersections)
        CHECK(avoided_swaths.size() >= input_swaths.size());
    }

    SUBCASE("Small inflation") {
        auto avoided_swaths = avoider.avoid(input_swaths, 1.0);
        // With small inflation, expect moderate changes
        CHECK(avoided_swaths.size() >= input_swaths.size());
    }

    SUBCASE("Large inflation") {
        auto avoided_swaths = avoider.avoid(input_swaths, 5.0);
        // With large inflation, expect more significant changes
        // (more swaths affected, potentially more splits)
        CHECK(avoided_swaths.size() >= input_swaths.size());
    }
}
