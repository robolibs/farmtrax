#include "doctest/doctest.h"
#include "farmtrax/divy.hpp"
#include "farmtrax/field.hpp"
#include <memory>

// Helper to create a test field with a single part
std::shared_ptr<farmtrax::Part> create_test_part(const concord::Datum &datum = concord::Datum{}) {
    std::cout << "Debug: Starting create_test_part" << std::endl;

    // Create a simple rectangular polygon
    concord::Polygon poly;
    std::cout << "Debug: Adding polygon points" << std::endl;
    poly.addPoint(concord::Point{0.0, 0.0, 0.0});
    poly.addPoint(concord::Point{100.0, 0.0, 0.0});
    poly.addPoint(concord::Point{100.0, 50.0, 0.0});
    poly.addPoint(concord::Point{0.0, 50.0, 0.0});
    poly.addPoint(concord::Point{0.0, 0.0, 0.0}); // Close the polygon
    std::cout << "Debug: Polygon created with " << poly.getPoints().size() << " points" << std::endl;

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
            std::cout << "Debug: Processing swath " << i << ", b_line size: " << swath.b_line.size() << std::endl;

            // Initialize centerline for each swath
            if (swath.b_line.size() > 0 && swath.centerline.empty()) {
                std::cout << "Debug: Initializing centerline" << std::endl;
                for (const auto &bpoint : swath.b_line) {
                    swath.centerline.push_back(bpoint);
                }
                std::cout << "Debug: Centerline initialized with " << swath.centerline.size() << " points" << std::endl;
            }

            // Make sure points are set
            if (swath.points.empty() && !swath.b_line.empty()) {
                std::cout << "Debug: Setting points" << std::endl;
                swath.points.reserve(swath.b_line.size());
                for (const auto &bpoint : swath.b_line) {
                    swath.points.push_back(farmtrax::utils::from_boost(bpoint, datum));
                }
                std::cout << "Debug: Points set with " << swath.points.size() << " points" << std::endl;
            }

            // Make sure line is set
            if (!swath.points.empty() && (swath.line.getStart().x == 0 && swath.line.getStart().y == 0 &&
                                          swath.line.getEnd().x == 0 && swath.line.getEnd().y == 0)) {
                std::cout << "Debug: Setting line" << std::endl;
                swath.line.setStart(swath.points.front());
                swath.line.setEnd(swath.points.back());
                std::cout << "Debug: Line set successfully" << std::endl;
            }

            // Ensure bounding box is computed
            if (swath.bounding_box.min_corner().x() == 0 && swath.bounding_box.min_corner().y() == 0 &&
                swath.bounding_box.max_corner().x() == 0 && swath.bounding_box.max_corner().y() == 0) {
                std::cout << "Debug: Computing bounding box" << std::endl;
                if (swath.b_line.empty()) {
                    std::cout << "Warning: Cannot compute bounding box - empty b_line" << std::endl;
                } else {
                    swath.bounding_box = boost::geometry::return_envelope<farmtrax::BBox>(swath.b_line);
                    std::cout << "Debug: Bounding box computed" << std::endl;
                }
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
    concord::Datum datum{51.0, 5.0, 0.0};
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
    concord::Datum datum{51.0, 5.0, 0.0};
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
    concord::Datum datum{51.0, 5.0, 0.0};
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
            avg_x_m0 += swath->line.getStart().x;
        }
        avg_x_m0 /= result.swaths_per_machine.at(0).size();

        for (const auto &swath : result.swaths_per_machine.at(1)) {
            // Use the line start point's x coordinate
            avg_x_m1 += swath->line.getStart().x;
        }
        avg_x_m1 /= result.swaths_per_machine.at(1).size();

        // In block division, the averages should differ significantly
        // (indicating spatial grouping)
        CHECK(std::abs(avg_x_m0 - avg_x_m1) > 5.0);
    }
}

TEST_CASE("Changing Machine Count") {
    concord::Datum datum{51.0, 5.0, 0.0};
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
