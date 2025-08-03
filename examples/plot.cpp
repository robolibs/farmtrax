#include "concord/concord.hpp"
#include "entropy/generator.hpp"
#include "farmtrax/avoid.hpp"
#include "farmtrax/divy.hpp"
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp"
#include "farmtrax/utils/visualize.hpp"
#include "geoget/geoget.hpp"
#include "rerun.hpp"
#include "rerun/recording_stream.hpp"
#include "zoneout/zoneout.hpp"
#include <iomanip>
#include <iostream>

zoneout::Plot create_field(const std::string &zone_name, const std::string &crop_type,
                           const concord::Datum &datum = concord::Datum{51.98776171041831, 5.662378206146002, 0.0}) {
    zoneout::Plot plot("Wageningen Farm", "agricultural", datum);
    plot.setProperty("farm_type", "research");
    plot.setProperty("owner", "Wageningen Research Labs");

    typedef std::unordered_map<std::string, std::string> Props;

    geoget::PolygonDrawer drawer(datum);
    if (drawer.start(8080)) {
        const auto polygons = drawer.get_polygons();
        drawer.stop();

        if (!polygons.empty()) {
            zoneout::Zone zone(zone_name, "field", polygons[0], datum, 0.1);

            const auto &base_grid = zone.grid_data_.getGrid(0).grid;
            auto temp_grid = base_grid;
            auto moisture_grid = base_grid;

            entropy::NoiseGen temp_noise, moisture_noise;
            temp_noise.SetNoiseType(entropy::NoiseGen::NoiseType_Perlin);
            temp_noise.SetFrequency(0.08f);
            temp_noise.SetSeed(std::random_device{}());

            moisture_noise.SetNoiseType(entropy::NoiseGen::NoiseType_OpenSimplex2);
            moisture_noise.SetFrequency(0.05f);
            moisture_noise.SetSeed(std::random_device{}() + 100);

            for (size_t r = 0; r < temp_grid.rows(); ++r) {
                for (size_t c = 0; c < temp_grid.cols(); ++c) {
                    float temp_noise_val = temp_noise.GetNoise(static_cast<float>(r), static_cast<float>(c));
                    uint8_t temp_value = static_cast<uint8_t>(15 + (temp_noise_val + 1.0f) * 0.5f * (35 - 15));
                    temp_grid.set_value(r, c, temp_value);

                    float moisture_noise_val = moisture_noise.GetNoise(static_cast<float>(r), static_cast<float>(c));
                    uint8_t moisture_value = static_cast<uint8_t>(20 + (moisture_noise_val + 1.0f) * 0.5f * (80 - 20));
                    moisture_grid.set_value(r, c, moisture_value);
                }
            }

            zone.addRasterLayer(temp_grid, "temperature", "environmental", {{"units", "celsius"}}, true);
            zone.addRasterLayer(moisture_grid, "moisture", "environmental", {{"units", "percentage"}}, true);

            zone.setProperty("crop_type", crop_type);
            zone.setProperty("planting_date", "2024-04-15");
            zone.setProperty("irrigation", "true");

            plot.addZone(zone);
            std::cout << "Added zone: " << zone.getName() << " (ID: " << zone.getId().toString() << ")" << std::endl;

            // Add remaining polygons as features to the zone that's now in the plot
            if (plot.getZoneCount() > 0) {
                auto &plot_zone = plot.getZones().back(); // Get the zone we just added

                for (size_t i = 1; i < polygons.size(); ++i) {
                    Props properties = {{"area_m2", std::to_string(static_cast<int>(polygons[i].area()))}};
                    std::string feature_name = zone_name + "_feature_" + std::to_string(i);
                    try {
                        plot_zone.addPolygonFeature(polygons[i], feature_name, "obstacle", "obstacle", properties);
                    } catch (const std::exception &e) {
                        std::cout << "Failed to add feature " << feature_name << ": " << e.what() << std::endl;
                    }
                }
            }
        }
    }
    return plot;
}

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("farmtrax", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }
    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    // auto farm = create_field("Pea_Field", "pea", concord::Datum{51.73019, 4.23883, 0.0});
    // farm.save("/home/bresilla/farm_plot_2");
    auto farm = zoneout::Plot::load("/home/bresilla/farm_plot", "Pea Farm", "agricultural");

    auto zones = farm.getZones();
    std::cout << "Num zones: " << zones.size() << std::endl;

    auto zone0 = zones.at(0);
    auto boundary = zone0.poly_data_.getFieldBoundary();
    std::cout << "Zone 0 boundary: " << boundary.getPoints().size() << " points" << std::endl;

    const auto &polygon_elements = zone0.poly_data_.getPolygonElements();
    auto obstacle = polygon_elements[0].geometry;
    std::cout << "Zone 0 obstacle: " << obstacle.getPoints().size() << " points" << std::endl;
    std::vector<concord::Polygon> obstacles;
    obstacle.addPoint(obstacle.getPoints().front());
    obstacles.push_back(obstacle);

    // Use equal-area partitioning with target of 80000 sq.m (8 hectares)
    farmtrax::Field field(boundary, farm.getDatum(), true, 100000.0, true);

    std::cout << "Field border points: " << field.get_border().getPoints().size() << std::endl;
    std::cout << "Field parts: " << field.get_parts().size() << std::endl;

    field.gen_field(4.0, 0.0, 3);
    auto num_machines = 3;

    farmtrax::ObstacleAvoider avoider(obstacles, farm.getDatum());

    farmtrax::visualize::show_obstacles(obstacles, rec);
    farmtrax::visualize::show_field(field, rec);

    for (size_t f = 0; f < field.get_parts().size(); f++) {

        const auto &part = field.get_parts()[f];

        auto part_area = boost::geometry::area(part.boundary.b_polygon);
        std::cout << "Part " << f << " area: " << part_area << " sq.m (" << part_area / 10000.0 << " hectares)"
                  << std::endl;

        auto fieldPtr = std::make_shared<farmtrax::Part>(field.get_parts()[f]);
        farmtrax::Divy divy(fieldPtr, farmtrax::DivisionType::ALTERNATE, num_machines);
        divy.compute_division();

        // farmtrax::visualize::show_divisions(divy, rec, f);

        auto &res = divy.result();
        for (std::size_t m = 0; m < num_machines; ++m) {
            if (res.swaths_per_machine.at(m).empty()) {
                std::cout << "Machine " << m << " has no swaths assigned\n";
                continue;
            }

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
