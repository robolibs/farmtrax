#include <iomanip>
#include <iostream>
#include <thread>

#include <concord/concord.hpp>
#include <datapod/datapod.hpp>

#include "rerun/recording_stream.hpp"

#include "farmtrax/avoid.hpp"
#include "farmtrax/divy.hpp"
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp"
#include "rerun.hpp"
#include "thread"

#include "farmtrax/utils/visualize.hpp"

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("farmtrax", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    // WGS84 coordinates for the field boundary
    std::vector<concord::earth::WGS> coordinates;
    coordinates.push_back(concord::earth::WGS(51.98765392402663, 5.660072928621929, 0.0));
    coordinates.push_back(concord::earth::WGS(51.98816428304869, 5.661754957062072, 0.0));
    coordinates.push_back(concord::earth::WGS(51.989850316694316, 5.660416700858434, 0.0));
    coordinates.push_back(concord::earth::WGS(51.990417354104295, 5.662166255987472, 0.0));
    coordinates.push_back(concord::earth::WGS(51.991078888673854, 5.660969191951295, 0.0));
    coordinates.push_back(concord::earth::WGS(51.989479848375254, 5.656874619070777, 0.0));
    coordinates.push_back(concord::earth::WGS(51.988156722216644, 5.657715633290422, 0.0));
    coordinates.push_back(concord::earth::WGS(51.98765392402663, 5.660072928621929, 0.0)); // Close

    // Define the world datum (reference point for ENU conversion)
    datapod::Geo world_datum{51.98954034749562, 5.6584737410504715, 53.801823};

    // Convert WGS84 to ENU coordinates
    datapod::Polygon poly;
    for (const auto &wgs_coord : coordinates) {
        auto enu = concord::frame::to_enu(world_datum, wgs_coord);
        poly.vertices.push_back(datapod::Point{enu.east(), enu.north(), enu.up()});
    }

    farmtrax::Field field(poly, world_datum, true, 100000.0);

    field.gen_field(4.0, 0.0, 3);
    auto num_machines = 2;

    // Create some example obstacles (e.g., trees, buildings, water bodies)
    std::vector<datapod::Polygon> obstacles;

    // Calculate the actual center of the field based on its bounds
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto &point : poly.vertices) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }

    double center_x = (min_x + max_x) / 2.0;
    double center_y = (min_y + max_y) / 2.0;

    std::cout << "Field bounds: x[" << min_x << ", " << max_x << "], y[" << min_y << ", " << max_y << "]\n";
    std::cout << "Field center: (" << center_x << ", " << center_y << ")\n";

    // Create a square obstacle at the center of the field
    datapod::Polygon obstacle1;
    double obstacle_size = 25.0; // 25 meter square obstacle

    obstacle1.vertices.push_back(datapod::Point{center_x - obstacle_size, center_y - obstacle_size, 0});
    obstacle1.vertices.push_back(datapod::Point{center_x + obstacle_size, center_y - obstacle_size, 0});
    obstacle1.vertices.push_back(datapod::Point{center_x + obstacle_size, center_y + obstacle_size, 0});
    obstacle1.vertices.push_back(datapod::Point{center_x - obstacle_size, center_y + obstacle_size, 0});
    obstacle1.vertices.push_back(datapod::Point{center_x - obstacle_size, center_y - obstacle_size, 0}); // Close

    // Add the obstacle to the obstacles vector
    obstacles.push_back(obstacle1);

    // Create obstacle avoider
    farmtrax::ObstacleAvoider avoider(obstacles, world_datum);

    std::cout << "Created " << obstacles.size() << " obstacles\n";

    // Visualize obstacles
    farmtrax::visualize::show_obstacles(obstacles, rec);

    auto part_cnt = field.get_parts().size();
    std::cout << "\n=== Field Processing with Area-Based Partitioning ===\n";
    std::cout << "Total field parts: " << part_cnt << "\n";

    farmtrax::visualize::show_field(field, rec, world_datum);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    for (size_t f = 0; f < field.get_parts().size(); f++) {
        std::cout << "\n--- Processing Field Part " << (f + 1) << " of " << part_cnt << " ---\n";

        const auto &part = field.get_parts()[f];

        // Calculate part area using datapod polygon
        auto part_area = std::abs(part.boundary.polygon.area());
        std::cout << "Part " << (f + 1) << ": " << std::fixed << std::setprecision(1) << part_area << " sq.m ("
                  << (part_area / 10000.0) << " hectares), " << part.headlands.size() << " headlands, "
                  << part.swaths.size() << " swaths\n";

        auto fieldPtr = std::make_shared<farmtrax::Part>(field.get_parts()[f]);
        farmtrax::Divy divy(fieldPtr, farmtrax::DivisionType::ALTERNATE, num_machines);
        divy.compute_division();

        // farmtrax::visualize::show_divisions(divy, rec, f);

        std::this_thread::sleep_for(std::chrono::seconds(10));

        num_machines = 4;
        divy.set_machine_count(num_machines);
        divy.compute_division();

        // farmtrax::visualize::show_divisions(divy, rec, f);

        auto &res = divy.result();
        for (std::size_t m = 0; m < num_machines; ++m) {
            if (res.swaths_per_machine.at(m).empty()) {
                std::cout << "Machine " << m << " has no swaths assigned\n";
                continue;
            }

            // Apply obstacle avoidance to the swaths
            std::cout << "Machine " << m << " original swaths: " << res.swaths_per_machine.at(m).size() << "\n";

            // Apply obstacle avoidance with 2.0 meter inflation distance
            auto avoided_swaths = avoider.avoid(res.swaths_per_machine.at(m), 2.0f);

            // Create Nety instance from obstacle-avoided swaths (now filters to only SwathType::Swath)
            farmtrax::Nety nety(avoided_swaths);
            nety.field_traversal(); // This reorders the swaths internally

            std::cout << "Machine " << m << " has " << nety.get_swaths().size()
                      << " swaths in Nety after filtering (only regular swaths)\n";

            // Visualize the optimized swath tour using the reordered swaths with part number
            farmtrax::visualize::show_swath_tour_for_part(nety, rec, f, m);
        }
    }

    return 0;
}
