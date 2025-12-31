#pragma once

#ifdef HAS_RERUN

#include <datapod/datapod.hpp>

#include "../avoid.hpp"
#include "../divy.hpp"
#include "../field.hpp"
#include "../graph.hpp"
#include "../turners/dubins.hpp"
#include "../turners/reeds_shepp.hpp"
#include "rerun.hpp"
#include <array>
#include <rerun/archetypes/geo_line_strings.hpp>
#include <rerun/components/geo_line_string.hpp>
#include <rerun/components/lat_lon.hpp>
#include <rerun/recording_stream.hpp>
#include <rerun/result.hpp>
#include <vector>

#include <concord/frame/convert.hpp>

namespace farmtrax {
    namespace visualize {

        /**
         * @brief Convert ENU point to WGS84 LatLon for geo visualization
         */
        inline rerun::LatLon enu_to_latlon(const datapod::Point &enu_pt, const datapod::Geo &datum) {
            // Create ENU with origin
            concord::frame::ENU enu{enu_pt.x, enu_pt.y, enu_pt.z, datum};
            // Convert to WGS
            auto wgs = concord::frame::to_wgs(enu);
            return rerun::LatLon{float(wgs.latitude), float(wgs.longitude)};
        }

        inline void show_field(const farmtrax::Field &field, std::shared_ptr<rerun::RecordingStream> rec,
                               const datapod::Geo &datum) {
            // Show the main field border - ENU coordinates
            std::vector<std::array<float, 3>> border_pts;
            std::vector<rerun::LatLon> border_wgs;
            for (auto const &p : field.get_border().vertices) {
                border_pts.push_back({float(p.x), float(p.y), 0.0f});
                border_wgs.push_back(enu_to_latlon(p, datum));
            }

            // Close the border polygon if not already closed
            if (!border_pts.empty() &&
                (border_pts.front()[0] != border_pts.back()[0] || border_pts.front()[1] != border_pts.back()[1])) {
                border_pts.push_back(border_pts.front());
                border_wgs.push_back(border_wgs.front());
            }

            std::cout << "Visualizing field border with " << border_pts.size() << " points" << std::endl;

            // Log ENU coordinates (3D view)
            rec->log_static("/field/border", rerun::LineStrips3D(rerun::components::LineStrip3D(border_pts))
                                                 .with_colors({{rerun::Color(120, 70, 70)}})
                                                 .with_radii({{0.2f}}));

            // Log WGS84 coordinates (Map view)
            auto geo_border = rerun::components::GeoLineString::from_lat_lon(border_wgs);
            rec->log_static("/field/border", rerun::archetypes::GeoLineStrings(geo_border)
                                                 .with_colors({{rerun::Color(120, 70, 70)}})
                                                 .with_radii({{0.5f}}));

            for (size_t i = 0; i < field.get_parts().size(); ++i) {
                // Headlands
                for (size_t j = 0; j < field.get_parts()[i].headlands.size(); ++j) {
                    std::vector<std::array<float, 3>> pts;
                    std::vector<rerun::LatLon> wgs_pts;
                    for (auto const &p : field.get_parts()[i].headlands[j].polygon.vertices) {
                        pts.push_back({float(p.x), float(p.y), 0.0f});
                        wgs_pts.push_back(enu_to_latlon(p, datum));
                    }
                    // Close polygon
                    if (!pts.empty() && (pts.front()[0] != pts.back()[0] || pts.front()[1] != pts.back()[1])) {
                        pts.push_back(pts.front());
                        wgs_pts.push_back(wgs_pts.front());
                    }

                    rec->log_static("/field/part" + std::to_string(i) + "/headland" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{rerun::Color(70, 120, 70)}})
                                        .with_radii({{0.2f}}));

                    auto geo_headland = rerun::components::GeoLineString::from_lat_lon(wgs_pts);
                    rec->log_static("/field/part" + std::to_string(i) + "/headland" + std::to_string(j),
                                    rerun::archetypes::GeoLineStrings(geo_headland)
                                        .with_colors({{rerun::Color(70, 120, 70)}})
                                        .with_radii({{0.5f}}));
                }

                // Swaths
                for (size_t j = 0; j < field.get_parts()[i].swaths.size(); ++j) {
                    auto const &s = field.get_parts()[i].swaths[j];
                    std::vector<std::array<float, 3>> pts = {{float(s.line.start.x), float(s.line.start.y), 0.0f},
                                                             {float(s.line.end.x), float(s.line.end.y), 0.0f}};
                    std::vector<rerun::LatLon> wgs_pts = {enu_to_latlon(s.line.start, datum),
                                                          enu_to_latlon(s.line.end, datum)};

                    rec->log_static("/field/part" + std::to_string(i) + "/swath" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{rerun::Color(70, 70, 120)}})
                                        .with_radii({{0.2f}}));

                    auto geo_swath = rerun::components::GeoLineString::from_lat_lon(wgs_pts);
                    rec->log_static("/field/part" + std::to_string(i) + "/swath" + std::to_string(j),
                                    rerun::archetypes::GeoLineStrings(geo_swath)
                                        .with_colors({{rerun::Color(70, 70, 120)}})
                                        .with_radii({{0.5f}}));
                }
            }
        }

        // Backward compatible version without datum (no geo visualization)
        inline void show_field(const farmtrax::Field &field, std::shared_ptr<rerun::RecordingStream> rec) {
            // Show the main field border - ENU coordinates only
            std::vector<std::array<float, 3>> border_pts;
            for (auto const &p : field.get_border().vertices)
                border_pts.push_back({float(p.x), float(p.y), 0.0f});

            if (!border_pts.empty() &&
                (border_pts.front()[0] != border_pts.back()[0] || border_pts.front()[1] != border_pts.back()[1])) {
                border_pts.push_back(border_pts.front());
            }

            std::cout << "Visualizing field border with " << border_pts.size() << " points" << std::endl;

            rec->log_static("/field/border", rerun::LineStrips3D(rerun::components::LineStrip3D(border_pts))
                                                 .with_colors({{rerun::Color(120, 70, 70)}})
                                                 .with_radii({{0.2f}}));
            for (size_t i = 0; i < field.get_parts().size(); ++i) {
                for (size_t j = 0; j < field.get_parts()[i].headlands.size(); ++j) {
                    std::vector<std::array<float, 3>> pts;
                    for (auto const &p : field.get_parts()[i].headlands[j].polygon.vertices)
                        pts.push_back({float(p.x), float(p.y), 0.0f});
                    rec->log_static("/field/part" + std::to_string(i) + "/headland" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{rerun::Color(70, 120, 70)}})
                                        .with_radii({{0.2f}}));
                }
            }
            for (size_t i = 0; i < field.get_parts().size(); ++i) {
                for (size_t j = 0; j < field.get_parts()[i].swaths.size(); ++j) {
                    auto const &s = field.get_parts()[i].swaths[j];
                    std::vector<std::array<float, 3>> pts = {{float(s.line.start.x), float(s.line.start.y), 0.0f},
                                                             {float(s.line.end.x), float(s.line.end.y), 0.0f}};
                    rec->log_static("/field/part" + std::to_string(i) + "/swath" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{rerun::Color(70, 70, 120)}})
                                        .with_radii({{0.2f}}));
                }
            }
        }

        inline void show_divisions(const Divy &divy, std::shared_ptr<rerun::RecordingStream> rec, float radius = 0.2f) {
            static const std::vector<rerun::Color> palette = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                                                              {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
            auto const &div = divy.result();
            for (auto const &kv : div.headlands_per_machine) {
                size_t m = kv.first;
                auto const &headlands = kv.second;
                auto color = palette[m % palette.size()];
                for (size_t j = 0; j < headlands.size(); ++j) {
                    auto ring_ptr = headlands[j];
                    std::vector<std::array<float, 3>> pts;
                    for (auto const &p : ring_ptr->polygon.vertices)
                        pts.push_back({float(p.x), float(p.y), 0.1f});
                    rec->log_static("/division/headland" + std::to_string(m) + "_" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{color}})
                                        .with_radii({{radius}}));
                }
            }
            for (auto const &kv : div.swaths_per_machine) {
                size_t m = kv.first;
                auto const &swaths = kv.second;
                auto color = palette[m % palette.size()];
                for (size_t j = 0; j < swaths.size(); ++j) {
                    auto sw_ptr = swaths[j];
                    std::vector<std::array<float, 3>> pts = {
                        {float(sw_ptr->line.start.x), float(sw_ptr->line.start.y), 0.1f},
                        {float(sw_ptr->line.end.x), float(sw_ptr->line.end.y), 0.0f}};
                    rec->log_static("/division/swath" + std::to_string(m) + "_" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{color}})
                                        .with_radii({{radius}}));
                }
            }
        }

        // Version with part number support
        inline void show_divisions(const Divy &divy, std::shared_ptr<rerun::RecordingStream> rec, size_t part_id,
                                   float radius = 0.2f) {
            static const std::vector<rerun::Color> palette = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                                                              {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
            auto const &div = divy.result();
            for (auto const &kv : div.headlands_per_machine) {
                size_t m = kv.first;
                auto const &headlands = kv.second;
                auto color = palette[m % palette.size()];
                for (size_t j = 0; j < headlands.size(); ++j) {
                    auto ring_ptr = headlands[j];
                    std::vector<std::array<float, 3>> pts;
                    for (auto const &p : ring_ptr->polygon.vertices)
                        pts.push_back({float(p.x), float(p.y), 0.1f});
                    rec->log_static("/part" + std::to_string(part_id) + "/division/headland" + std::to_string(m) + "_" +
                                        std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{color}})
                                        .with_radii({{radius}}));
                }
            }
            for (auto const &kv : div.swaths_per_machine) {
                size_t m = kv.first;
                auto const &swaths = kv.second;
                auto color = palette[m % palette.size()];
                for (size_t j = 0; j < swaths.size(); ++j) {
                    auto sw_ptr = swaths[j];
                    std::vector<std::array<float, 3>> pts = {
                        {float(sw_ptr->line.start.x), float(sw_ptr->line.start.y), 0.1f},
                        {float(sw_ptr->line.end.x), float(sw_ptr->line.end.y), 0.0f}};
                    rec->log_static("/part" + std::to_string(part_id) + "/division/swath" + std::to_string(m) + "_" +
                                        std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{color}})
                                        .with_radii({{radius}}));
                }
            }
        }

        // Version that takes swaths directly (assumes they are already in the correct traversal order)
        inline void show_swath_tour(const std::vector<std::shared_ptr<const Swath>> &swaths,
                                    std::shared_ptr<rerun::RecordingStream> rec, size_t machine_id = 0,
                                    float swath_radius = 0.4f, float connection_radius = 0.2f) {
            static const std::vector<rerun::Color> palette = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                                                              {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};

            auto base_color = palette[machine_id % palette.size()];
            auto connection_color = rerun::Color(base_color.r() * 0.6, base_color.g() * 0.6, base_color.b() * 0.6);

            if (swaths.empty())
                return;

            // Visualize each swath based on its type
            for (size_t i = 0; i < swaths.size(); ++i) {
                const auto &swath = swaths[i];

                // Choose color and radius based on swath type
                rerun::Color swath_color;
                float radius;
                std::string prefix;

                if (swath->type == SwathType::Connection) {
                    swath_color = connection_color;
                    radius = connection_radius;
                    prefix = "connection";
                } else {
                    swath_color = base_color;
                    radius = swath_radius;
                    prefix = "swath";
                }

                // Visualize the swath line
                std::vector<std::array<float, 3>> line_points = {
                    {float(swath->getHead().x), float(swath->getHead().y), 0.3f},
                    {float(swath->getTail().x), float(swath->getTail().y), 0.3f}};

                rec->log_static("/tour/machine" + std::to_string(machine_id) + "/" + prefix + std::to_string(i),
                                rerun::LineStrips3D(rerun::components::LineStrip3D(line_points))
                                    .with_colors({{swath_color}})
                                    .with_radii({{radius}}));
            }

            // Add machine start and end point markers
            if (!swaths.empty()) {
                const auto &first_swath = swaths[0];
                const auto &last_swath = swaths[swaths.size() - 1];

                std::vector<rerun::Position3D> start_point = {
                    {float(first_swath->getHead().x), float(first_swath->getHead().y), 0.6f}};

                std::vector<rerun::Position3D> end_point = {
                    {float(last_swath->getTail().x), float(last_swath->getTail().y), 0.6f}};

                rec->log_static("/tour/machine" + std::to_string(machine_id) + "/start_marker",
                                rerun::Points3D(start_point)
                                    .with_colors({base_color}) // Same color as machine
                                    .with_radii({2.0f}));      // Larger radius for visibility

                rec->log_static("/tour/machine" + std::to_string(machine_id) + "/end_marker",
                                rerun::Points3D(end_point)
                                    .with_colors({base_color}) // Same color as machine
                                    .with_radii({2.0f}));      // Larger radius for visibility
            }
        }

        // Version that takes swaths with part number support
        inline void show_swath_tour(const std::vector<std::shared_ptr<const Swath>> &swaths,
                                    std::shared_ptr<rerun::RecordingStream> rec, size_t part_id, size_t machine_id = 0,
                                    float swath_radius = 0.4f, float connection_radius = 0.2f) {
            static const std::vector<rerun::Color> palette = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                                                              {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};

            auto base_color = palette[machine_id % palette.size()];
            auto connection_color = rerun::Color(base_color.r() * 0.6, base_color.g() * 0.6, base_color.b() * 0.6);

            if (swaths.empty())
                return;

            // Visualize each swath based on its type
            for (size_t i = 0; i < swaths.size(); ++i) {
                const auto &swath = swaths[i];

                // Choose color and radius based on swath type
                rerun::Color swath_color;
                float radius;
                std::string prefix;

                if (swath->type == SwathType::Connection) {
                    swath_color = connection_color;
                    radius = connection_radius;
                    prefix = "connection";
                } else {
                    swath_color = base_color;
                    radius = swath_radius;
                    prefix = "swath";
                }

                // Visualize the swath line
                std::vector<std::array<float, 3>> line_points = {
                    {float(swath->getHead().x), float(swath->getHead().y), 0.3f},
                    {float(swath->getTail().x), float(swath->getTail().y), 0.3f}};

                rec->log_static("/part" + std::to_string(part_id) + "/tour/machine" + std::to_string(machine_id) + "/" +
                                    prefix + std::to_string(i),
                                rerun::LineStrips3D(rerun::components::LineStrip3D(line_points))
                                    .with_colors({{swath_color}})
                                    .with_radii({{radius}}));
            }

            // Add machine start and end point markers
            if (!swaths.empty()) {
                const auto &first_swath = swaths[0];
                const auto &last_swath = swaths[swaths.size() - 1];

                std::vector<rerun::Position3D> start_point = {
                    {float(first_swath->getHead().x), float(first_swath->getHead().y), 0.6f}};

                std::vector<rerun::Position3D> end_point = {
                    {float(last_swath->getTail().x), float(last_swath->getTail().y), 0.6f}};

                rec->log_static("/part" + std::to_string(part_id) + "/tour/machine" + std::to_string(machine_id) +
                                    "/start_marker",
                                rerun::Points3D(start_point)
                                    .with_colors({base_color}) // Same color as machine
                                    .with_radii({2.0f}));      // Larger radius for visibility

                rec->log_static("/part" + std::to_string(part_id) + "/tour/machine" + std::to_string(machine_id) +
                                    "/end_marker",
                                rerun::Points3D(end_point)
                                    .with_colors({base_color}) // Same color as machine
                                    .with_radii({2.0f}));      // Larger radius for visibility
            }
        }

        // Convenience overload that uses the reordered swaths from Nety
        inline void show_swath_tour(const farmtrax::Nety &nety, std::shared_ptr<rerun::RecordingStream> rec,
                                    size_t machine_id = 0, float swath_radius = 0.4f, float connection_radius = 0.2f) {
            show_swath_tour(nety.get_swaths(), rec, machine_id, swath_radius, connection_radius);
        }

        // Convenience overload with part number support
        inline void show_swath_tour_for_part(const farmtrax::Nety &nety, std::shared_ptr<rerun::RecordingStream> rec,
                                             size_t part_id, size_t machine_id, float swath_radius = 0.4f,
                                             float connection_radius = 0.2f) {
            show_swath_tour(nety.get_swaths(), rec, part_id, machine_id, swath_radius, connection_radius);
        }

        // Visualize obstacles as border-only polygons
        inline void show_obstacles(const std::vector<datapod::Polygon> &obstacles,
                                   std::shared_ptr<rerun::RecordingStream> rec, float height = 0.5f) {
            for (size_t i = 0; i < obstacles.size(); ++i) {
                const auto &obstacle = obstacles[i];

                // Create vertices for the polygon
                std::vector<std::array<float, 3>> vertices;
                for (const auto &point : obstacle.vertices) {
                    vertices.push_back({float(point.x), float(point.y), height});
                }

                // Create outline only (no filled polygon)
                rec->log_static("/obstacles/outline" + std::to_string(i),
                                rerun::LineStrips3D(rerun::components::LineStrip3D(vertices))
                                    .with_colors({{rerun::Color(220, 20, 20)}}) // Red for obstacles
                                    .with_radii({{0.5f}})); // Slightly thicker for better visibility
            }
        }

        // Visualize inflated obstacles (for debugging)
        inline void show_inflated_obstacles(const std::vector<datapod::Polygon> &inflated_obstacles,
                                            std::shared_ptr<rerun::RecordingStream> rec, float height = 0.6f) {
            for (size_t i = 0; i < inflated_obstacles.size(); ++i) {
                const auto &obstacle = inflated_obstacles[i];

                // Create vertices for the inflated polygon
                std::vector<std::array<float, 3>> vertices;
                for (const auto &point : obstacle.vertices) {
                    vertices.push_back({float(point.x), float(point.y), height});
                }

                // Create outline for inflated obstacle
                rec->log_static("/obstacles/inflated_outline" + std::to_string(i),
                                rerun::LineStrips3D(rerun::components::LineStrip3D(vertices))
                                    .with_colors({{rerun::Color(255, 150, 150)}}) // Light red for inflated area
                                    .with_radii({{0.2f}}));
            }
        }

        // Visualize avoided swaths with different colors for different types
        inline void show_avoided_swaths(const std::vector<std::shared_ptr<Swath>> &avoided_swaths,
                                        std::shared_ptr<rerun::RecordingStream> rec, size_t machine_id = 0,
                                        const std::string &prefix = "avoided") {
            static const std::vector<rerun::Color> palette = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                                                              {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};

            auto base_color = palette[machine_id % palette.size()];

            for (size_t i = 0; i < avoided_swaths.size(); ++i) {
                const auto &swath = avoided_swaths[i];

                // Choose visualization based on swath type
                rerun::Color swath_color;
                float radius;
                float height;
                std::string type_prefix;

                if (swath->type == SwathType::Around) {
                    // Connection/around swaths - use orange/yellow
                    swath_color = rerun::Color(255, 165, 0); // Orange
                    radius = 0.3f;
                    height = 0.4f;
                    type_prefix = "connection";
                } else if (swath->type == SwathType::Swath) {
                    // Regular working swaths
                    swath_color = base_color;
                    radius = 0.4f;
                    height = 0.2f;
                    type_prefix = "regular";
                } else {
                    // Other types
                    swath_color = rerun::Color(base_color.r() * 0.8, base_color.g() * 0.8, base_color.b() * 0.8);
                    radius = 0.3f;
                    height = 0.3f;
                    type_prefix = "other";
                }

                // Visualize the swath line
                std::vector<std::array<float, 3>> line_points = {
                    {float(swath->getHead().x), float(swath->getHead().y), height},
                    {float(swath->getTail().x), float(swath->getTail().y), height}};

                rec->log_static("/" + prefix + "/machine" + std::to_string(machine_id) + "/" + type_prefix +
                                    std::to_string(i),
                                rerun::LineStrips3D(rerun::components::LineStrip3D(line_points))
                                    .with_colors({{swath_color}})
                                    .with_radii({{radius}}));

                // Add direction arrow at the start of each swath
                auto start = swath->getHead();
                auto end = swath->getTail();

                // Calculate direction vector
                float dx = end.x - start.x;
                float dy = end.y - start.y;
                float length = std::sqrt(dx * dx + dy * dy);

                if (length > 1e-6) { // Avoid division by zero
                    dx /= length;
                    dy /= length;

                    // Create arrow at start point
                    float arrow_length = 2.0f;
                    std::array<float, 3> arrow_tip = {float(start.x + dx * arrow_length),
                                                      float(start.y + dy * arrow_length), height + 0.1f};
                    std::array<float, 3> arrow_base = {float(start.x), float(start.y), height + 0.1f};

                    rec->log_static("/" + prefix + "/machine" + std::to_string(machine_id) + "/" + type_prefix +
                                        "_arrow" + std::to_string(i),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D({arrow_base, arrow_tip}))
                                        .with_colors({{swath_color}})
                                        .with_radii({{radius * 0.5f}}));
                }
            }
        }

        // Combined visualization function for obstacle avoidance
        inline void show_obstacle_avoidance(const farmtrax::ObstacleAvoider &avoider,
                                            const std::vector<std::shared_ptr<const Swath>> &original_swaths,
                                            const std::vector<std::shared_ptr<Swath>> &avoided_swaths,
                                            std::shared_ptr<rerun::RecordingStream> rec, size_t machine_id = 0) {
            // Show original obstacles (already datapod::Polygon)
            show_obstacles(avoider.get_obstacles(), rec);

            // Show inflated obstacles
            show_inflated_obstacles(avoider.get_inflated_obstacles(), rec);

            // Show original swaths (dimmed)
            for (size_t i = 0; i < original_swaths.size(); ++i) {
                const auto &swath = original_swaths[i];
                std::vector<std::array<float, 3>> line_points = {
                    {float(swath->getHead().x), float(swath->getHead().y), 0.1f},
                    {float(swath->getTail().x), float(swath->getTail().y), 0.1f}};

                rec->log_static("/original/machine" + std::to_string(machine_id) + "/swath" + std::to_string(i),
                                rerun::LineStrips3D(rerun::components::LineStrip3D(line_points))
                                    .with_colors({{rerun::Color(100, 100, 100, 100)}}) // Dimmed gray
                                    .with_radii({{0.1f}}));
            }

            // Show avoided swaths
            show_avoided_swaths(avoided_swaths, rec, machine_id, "avoided");
        }

        // Show Dubins path
        inline void show_dubins_path(std::shared_ptr<rerun::RecordingStream> rec,
                                     const turners::DubinsPath &dubins_path,
                                     const std::string &entity_path = "dubins_path",
                                     const rerun::Color &color = {255, 0, 0}) {
            if (dubins_path.waypoints.empty())
                return;

            std::vector<rerun::Position3D> positions;
            for (const auto &waypoint : dubins_path.waypoints) {
                positions.emplace_back(waypoint.point.x, waypoint.point.y, 0.0f);
            }

            if (!positions.empty()) {
                rec->log_static(entity_path,
                                rerun::LineStrips3D(rerun::components::LineStrip3D(positions)).with_colors(color));
            }

            // Show segment types as text annotations
            if (!dubins_path.segments.empty()) {
                std::string segment_info = dubins_path.name + " [";
                for (size_t i = 0; i < dubins_path.segments.size(); ++i) {
                    if (i > 0)
                        segment_info += ", ";
                    char seg_char = static_cast<char>(dubins_path.segments[i].type);
                    segment_info += seg_char;
                }
                segment_info += "]";

                rec->log_static(entity_path + "/info",
                                rerun::TextLog(segment_info).with_level(rerun::TextLogLevel::Info));
            }
        }

        // Show multiple Dubins paths (all in gray)
        inline void show_dubins_paths(std::shared_ptr<rerun::RecordingStream> rec,
                                      const std::vector<turners::DubinsPath> &dubins_paths,
                                      const std::string &base_entity_path = "dubins_paths") {
            rerun::Color gray_color(128, 128, 128); // Gray color for all paths
            for (size_t i = 0; i < dubins_paths.size(); ++i) {
                show_dubins_path(rec, dubins_paths[i], base_entity_path + "/" + dubins_paths[i].name, gray_color);
            }
        }

        // Show Reeds-Shepp path
        inline void show_reeds_shepp_path(std::shared_ptr<rerun::RecordingStream> rec,
                                          const turners::ReedsSheppPath &rs_path,
                                          const std::string &entity_path = "reeds_shepp_path",
                                          const rerun::Color &color = {0, 0, 255}) {
            if (rs_path.waypoints.empty())
                return;

            std::vector<rerun::Position3D> positions;
            std::vector<rerun::Color> colors;

            for (size_t i = 0; i < rs_path.waypoints.size(); ++i) {
                const auto &waypoint = rs_path.waypoints[i];
                positions.emplace_back(waypoint.point.x, waypoint.point.y, 0.0f);

                // Color code based on forward/backward motion
                // Find which segment this waypoint belongs to
                bool is_forward = true;
                if (!rs_path.segments.empty()) {
                    // Simple heuristic: assume equal distribution of waypoints across segments
                    size_t seg_idx = (i * rs_path.segments.size()) / rs_path.waypoints.size();
                    seg_idx = std::min(seg_idx, rs_path.segments.size() - 1);
                    is_forward = rs_path.segments[seg_idx].forward;
                }

                colors.emplace_back(is_forward ? color.r() : color.r() / 2, is_forward ? color.g() : color.g() / 2,
                                    is_forward ? color.b() : color.b() / 2);
            }

            rec->log_static(entity_path,
                            rerun::LineStrips3D(rerun::components::LineStrip3D(positions)).with_colors(colors));

            // Show segment types as text annotations
            if (!rs_path.segments.empty()) {
                std::string segment_info = rs_path.name + " [";
                for (size_t i = 0; i < rs_path.segments.size(); ++i) {
                    if (i > 0)
                        segment_info += ", ";
                    char seg_char = static_cast<char>(rs_path.segments[i].type);
                    segment_info += seg_char;
                    if (!rs_path.segments[i].forward)
                        segment_info += "(rev)";
                }
                segment_info += "]";

                rec->log_static(entity_path + "/info",
                                rerun::TextLog(segment_info).with_level(rerun::TextLogLevel::Info));
            }
        }

        // Show multiple Reeds-Shepp paths (all in gray)
        inline void show_reeds_shepp_paths(std::shared_ptr<rerun::RecordingStream> rec,
                                           const std::vector<turners::ReedsSheppPath> &rs_paths,
                                           const std::string &base_entity_path = "reeds_shepp_paths") {
            rerun::Color gray_color(128, 128, 128); // Gray color for all paths
            for (size_t i = 0; i < rs_paths.size(); ++i) {
                show_reeds_shepp_path(rec, rs_paths[i], base_entity_path + "/" + rs_paths[i].name, gray_color);
            }
        }

        // Show robot pose (using Pose2D from turners)
        inline void show_robot_pose(std::shared_ptr<rerun::RecordingStream> rec, const turners::Pose2D &pose,
                                    const std::string &entity_path = "robot", const rerun::Color &color = {255, 255, 0},
                                    float scale = 1.0f) {
            // Robot position
            rec->log_static(entity_path + "/position", rerun::Points3D({{static_cast<float>(pose.point.x),
                                                                         static_cast<float>(pose.point.y), 0.0f}})
                                                           .with_colors(color)
                                                           .with_radii(0.02f * scale));

            // Robot orientation (arrow)
            float arrow_length = 0.5f * scale;
            float end_x = pose.point.x + arrow_length * std::cos(pose.yaw);
            float end_y = pose.point.y + arrow_length * std::sin(pose.yaw);

            std::vector<rerun::Position3D> orientation_line = {
                {static_cast<float>(pose.point.x), static_cast<float>(pose.point.y), 0.0f}, {end_x, end_y, 0.0f}};
            rec->log_static(entity_path + "/orientation",
                            rerun::LineStrips3D(rerun::components::LineStrip3D(orientation_line)).with_colors(color));
        }

    } // namespace visualize
} // namespace farmtrax

#endif
