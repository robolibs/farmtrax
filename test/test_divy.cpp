#include "doctest/doctest.h"
#include "farmtrax/divy.hpp"
#include "farmtrax/field.hpp"
#include <memory>

// Helper function to create AABB from a Segment
inline datapod::AABB segment_to_aabb(const datapod::Segment &seg) {
    return datapod::AABB{datapod::Point{std::min(seg.start.x, seg.end.x), std::min(seg.start.y, seg.end.y),
                                        std::min(seg.start.z, seg.end.z)},
                         datapod::Point{std::max(seg.start.x, seg.end.x), std::max(seg.start.y, seg.end.y),
                                        std::max(seg.start.z, seg.end.z)}};
}

// Helper to check if AABB is valid (not default-initialized)
inline bool is_aabb_valid(const datapod::AABB &aabb) {
    return aabb.min_point.x != 0.0 || aabb.min_point.y != 0.0 || aabb.max_point.x != 0.0 || aabb.max_point.y != 0.0;
}

// Helper to create a test field with a single part
std::shared_ptr<farmtrax::Part> create_test_part(const datapod::Geo &datum = datapod::Geo{}) {
    std::cout << "Debug: Starting create_test_part" << std::endl;

    // Create a simple rectangular polygon
    datapod::Polygon poly;
    std::cout << "Debug: Adding polygon points" << std::endl;
    poly.vertices.push_back(datapod::Point{0.0, 0.0, 0.0});
    poly.vertices.push_back(datapod::Point{100.0, 0.0, 0.0});
    poly.vertices.push_back(datapod::Point{100.0, 50.0, 0.0});
    poly.vertices.push_back(datapod::Point{0.0, 50.0, 0.0});
    poly.vertices.push_back(datapod::Point{0.0, 0.0, 0.0}); // Close the polygon
    std::cout << "Debug: Polygon created with " << poly.vertices.size() << " points" << std::endl;

    // Create a field with the polygon - use large area threshold to avoid excessive partitioning
    std::cout << "Debug: Creating field" << std::endl;
    farmtrax::Field field(poly, datum, true, 10000.0); // Large area threshold
    std::cout << "Debug: Field created" << std::endl;

    // Generate field with swaths - use 0 headlands to avoid buffer operation issues
    std::cout << "Debug: Generating field" << std::endl;
    field.gen_field(10.0, 90.0, 0); // 10m track width, vertical angle (90 degrees), 0 headlands
    std::cout << "Debug: Field generated with " << field.get_parts().size() << " parts" << std::endl;

    // Return a copy of the first part
    if (!field.get_parts().empty()) {
        std::cout << "Debug: Creating shared_ptr for part" << std::endl;
        auto part = std::make_shared<farmtrax::Part>(field.get_parts()[0]);
        std::cout << "Debug: Part created, has " << part->swaths.size() << " swaths" << std::endl;

        // Ensure each swath is properly initialized
        std::cout << "Debug: Initializing swaths" << std::endl;
        for (size_t i = 0; i < part->swaths.size(); i++) {
            auto &swath = part->swaths[i];
            std::cout << "Debug: Processing swath " << i << std::endl;

            // Make sure points are set from line
            if (swath.points.empty()) {
                std::cout << "Debug: Setting points from line" << std::endl;
                swath.points.push_back(swath.line.start);
                swath.points.push_back(swath.line.end);
                std::cout << "Debug: Points set with " << swath.points.size() << " points" << std::endl;
            }

            // Ensure bounding box is computed
            if (!is_aabb_valid(swath.bounding_box)) {
                std::cout << "Debug: Computing bounding box" << std::endl;
                swath.bounding_box = segment_to_aabb(swath.line);
                std::cout << "Debug: Bounding box computed" << std::endl;
            }
        }

        std::cout << "Debug: Part initialization complete" << std::endl;
        return part;
    }

    std::cout << "Error: Field has no parts!" << std::endl;
    // Should never happen with valid input
    throw std::runtime_error("Failed to create test part");
}

TEST_CASE("Divy Constructor and Basic Parameters") {
    datapod::Geo datum{51.0, 5.0, 0.0};
    std::cout << "Creating test part..." << std::endl;
    auto part = create_test_part(datum);
    std::cout << "Test part created successfully" << std::endl;

    // Debug part structure
    std::cout << "Part has " << part->headlands.size() << " headlands" << std::endl;
    std::cout << "Part has " << part->swaths.size() << " swaths" << std::endl;
    std::cout << "Part internal pointer: " << part.get() << std::endl;

    // Test constructor
    std::cout << "Creating Divy object..." << std::endl;
    farmtrax::Divy divy(part, farmtrax::DivisionType::ALTERNATE, 2);
    std::cout << "Divy object created successfully" << std::endl;

    // Test parameter modification
    std::cout << "Setting machine count..." << std::endl;
    divy.set_machine_count(4);
    std::cout << "Setting division type..." << std::endl;
    divy.set_division_type(farmtrax::DivisionType::BLOCK);

    // Check invalid parameters
    std::cout << "Checking invalid parameters..." << std::endl;
    CHECK_THROWS_AS(divy.set_machine_count(0), std::invalid_argument);

    // Initially, division result should be empty
    std::cout << "Getting result..." << std::endl;
    const auto &result = divy.result();
    std::cout << "Checking result..." << std::endl;
    CHECK(result.swaths_per_machine.empty());
    std::cout << "Test completed successfully" << std::endl;
}

TEST_CASE("Division Computation with Alternating Assignment") {
    datapod::Geo datum{51.0, 5.0, 0.0};
    auto part = create_test_part(datum);

    // Create divider with alternating assignment
    farmtrax::Divy divy(part, farmtrax::DivisionType::ALTERNATE, 2);

    // Compute the division
    divy.compute_division();

    // Check the result
    const auto &result = divy.result();

    // Should have entries for both machines
    CHECK(result.swaths_per_machine.count(0) > 0);
    CHECK(result.swaths_per_machine.count(1) > 0);

    // Total swaths should equal the original number of swaths
    size_t total_assigned_swaths = 0;
    for (const auto &[machine_id, swaths] : result.swaths_per_machine) {
        total_assigned_swaths += swaths.size();
    }

    size_t original_swath_count = 0;
    for (const auto &swath : part->swaths) {
        if (swath.type == farmtrax::SwathType::Swath) {
            original_swath_count++;
        }
    }

    CHECK(total_assigned_swaths == original_swath_count);
}

TEST_CASE("Division Computation with Block Assignment") {
    datapod::Geo datum{51.0, 5.0, 0.0};
    auto part = create_test_part(datum);

    // All initialization should be done in the create_test_part function

    // Create divider with block assignment
    farmtrax::Divy divy(part, farmtrax::DivisionType::BLOCK, 2);

    // Compute the division
    divy.compute_division();

    // Check the result
    const auto &result = divy.result();

    // Should have entries for both machines
    CHECK(result.swaths_per_machine.count(0) > 0);
    CHECK(result.swaths_per_machine.count(1) > 0);

    // Check spatial continuity - swaths for a machine should be in contiguous blocks
    // This is hard to test precisely, but we can check for rough clustering
    if (!result.swaths_per_machine.at(0).empty() && !result.swaths_per_machine.at(1).empty()) {
        // Get average X position for each machine's swaths
        double avg_x_m0 = 0.0;
        double avg_x_m1 = 0.0;

        for (const auto &swath : result.swaths_per_machine.at(0)) {
            // Use the line start point's x coordinate
            avg_x_m0 += swath->line.start.x;
        }
        avg_x_m0 /= result.swaths_per_machine.at(0).size();

        for (const auto &swath : result.swaths_per_machine.at(1)) {
            // Use the line start point's x coordinate
            avg_x_m1 += swath->line.start.x;
        }
        avg_x_m1 /= result.swaths_per_machine.at(1).size();

        // In block division, the averages should differ significantly
        // (indicating spatial grouping)
        CHECK(std::abs(avg_x_m0 - avg_x_m1) > 5.0);
    }
}

TEST_CASE("Changing Machine Count") {
    datapod::Geo datum{51.0, 5.0, 0.0};
    auto part = create_test_part(datum);

    // Create divider with 2 machines
    farmtrax::Divy divy(part, farmtrax::DivisionType::ALTERNATE, 2);
    divy.compute_division();

    // Check initial division
    const auto &result1 = divy.result();
    CHECK(result1.swaths_per_machine.size() == 2);

    // Change to 3 machines
    divy.set_machine_count(3);
    divy.compute_division();

    // Check new division
    const auto &result2 = divy.result();
    CHECK(result2.swaths_per_machine.size() == 3);

    // Total assigned swaths should remain the same
    size_t total_swaths1 = 0;
    for (const auto &[machine_id, swaths] : result1.swaths_per_machine) {
        total_swaths1 += swaths.size();
    }

    size_t total_swaths2 = 0;
    for (const auto &[machine_id, swaths] : result2.swaths_per_machine) {
        total_swaths2 += swaths.size();
    }

    CHECK(total_swaths1 == total_swaths2);
}
