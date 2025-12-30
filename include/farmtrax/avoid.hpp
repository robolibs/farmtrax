#pragma once

#include "farmtrax/field.hpp"
#include "farmtrax/utils/utils.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/touches.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <memory>
#include <vector>

namespace farmtrax {

    class ObstacleAvoider {
        std::vector<BPolygon> obstacles_;
        std::vector<BPolygon> inflated_obstacles_;
        float inflation_distance_;
        concord::Datum datum_;

      public:
        /// Constructor takes array of polygons as obstacles
        inline explicit ObstacleAvoider(const std::vector<BPolygon> &obstacles,
                                        const concord::Datum &datum = concord::Datum{})
            : obstacles_(obstacles), inflation_distance_(0.0f), datum_(datum) {}

        /// Constructor with concord::Polygon obstacles
        inline explicit ObstacleAvoider(const std::vector<concord::Polygon> &obstacles,
                                        const concord::Datum &datum = concord::Datum{})
            : datum_(datum) {
            obstacles_.reserve(obstacles.size());
            for (const auto &poly : obstacles) {
                obstacles_.push_back(utils::to_boost(poly));
            }
            inflation_distance_ = 0.0f;
        }

        /// Main avoidance method
        inline std::vector<std::shared_ptr<Swath>> avoid(const std::vector<std::shared_ptr<const Swath>> &input_swaths,
                                                         float inflation_distance) {
            inflation_distance_ = inflation_distance;
            inflate_obstacles();

            std::vector<std::shared_ptr<Swath>> result_swaths;

            // Skip processing if no input swaths
            if (input_swaths.empty()) {
                return result_swaths;
            }

            for (const auto &swath : input_swaths) {
                if (!swath)
                    continue; // Skip null swaths

                auto processed_swaths = process_swath(swath);
                result_swaths.insert(result_swaths.end(), processed_swaths.begin(), processed_swaths.end());
            }

            return result_swaths;
        }

        /// Overload for non-const swaths
        inline std::vector<std::shared_ptr<Swath>> avoid(const std::vector<std::shared_ptr<Swath>> &input_swaths,
                                                         float inflation_distance) {
            std::vector<std::shared_ptr<const Swath>> const_swaths;
            const_swaths.reserve(input_swaths.size());
            for (const auto &swath : input_swaths) {
                const_swaths.push_back(swath);
            }
            return avoid(const_swaths, inflation_distance);
        }

        /// Get original obstacles
        inline const std::vector<BPolygon> &get_obstacles() const { return obstacles_; }

        /// Get inflated obstacles
        inline const std::vector<BPolygon> &get_inflated_obstacles() const { return inflated_obstacles_; }

      private:
        /// Inflate all obstacles by the given distance
        inline void inflate_obstacles() {
            inflated_obstacles_.clear();
            inflated_obstacles_.reserve(obstacles_.size());

            for (const auto &obstacle : obstacles_) {
                BPolygon inflated = inflate_polygon(obstacle, inflation_distance_);
                if (!boost::geometry::is_empty(inflated)) {
                    inflated_obstacles_.push_back(inflated);
                }
            }
        }

        /// Inflate a single polygon
        inline BPolygon inflate_polygon(const BPolygon &polygon, float distance) const {
            boost::geometry::model::multi_polygon<BPolygon> buffered;

            // Buffer strategy components
            boost::geometry::strategy::buffer::distance_symmetric<double> dist_strategy(distance);
            boost::geometry::strategy::buffer::side_straight side_strategy;
            boost::geometry::strategy::buffer::join_miter join_strategy;
            boost::geometry::strategy::buffer::end_flat end_strategy;
            boost::geometry::strategy::buffer::point_square point_strategy;

            try {
                boost::geometry::buffer(polygon, buffered, dist_strategy, side_strategy, join_strategy, end_strategy,
                                        point_strategy);
            } catch (const std::exception &e) {
                std::cerr << "Buffer operation failed: " << e.what() << std::endl;
                return polygon; // Return original if buffering fails
            }

            // Return the largest polygon from the multi_polygon result
            if (!buffered.empty()) {
                const BPolygon *largest = &buffered.front();
                double max_area = boost::geometry::area(*largest);

                for (const auto &poly : buffered) {
                    double area = boost::geometry::area(poly);
                    if (area > max_area) {
                        max_area = area;
                        largest = &poly;
                    }
                }
                return *largest;
            }

            return polygon; // Return original if no result
        }

        /// Process a single swath against all obstacles
        inline std::vector<std::shared_ptr<Swath>> process_swath(const std::shared_ptr<const Swath> &swath) {
            std::vector<std::shared_ptr<Swath>> result;

            if (!swath) {
                return result;
            }

            // Convert swath to boost linestring
            BLineString swath_line = utils::to_boost(swath->line);

            // Check if swath intersects with any obstacle
            bool intersects = false;
            for (const auto &obstacle : inflated_obstacles_) {
                if (boost::geometry::intersects(swath_line, obstacle)) {
                    intersects = true;
                    break;
                }
            }

            if (!intersects) {
                // No intersection, keep original swath
                result.push_back(std::make_shared<Swath>(*swath));
            } else {
                // Process intersection and create cut swaths
                auto cut_swaths = cut_swath_around_obstacles(swath);
                result.insert(result.end(), cut_swaths.begin(), cut_swaths.end());
            }

            return result;
        }

        /// Cut swath around obstacles
        inline std::vector<std::shared_ptr<Swath>>
        cut_swath_around_obstacles(const std::shared_ptr<const Swath> &swath) {
            std::vector<std::shared_ptr<Swath>> result;

            if (!swath) {
                return result;
            }

            BLineString swath_line = utils::to_boost(swath->line);

            // For each obstacle, check intersection and cut the swath
            for (const auto &obstacle : inflated_obstacles_) {
                std::vector<BLineString> differences;
                try {
                    boost::geometry::difference(swath_line, obstacle, differences);

                    // Convert back to Swath objects
                    for (const auto &diff_line : differences) {
                        if (diff_line.size() >= 2) {
                            concord::Point start_point{diff_line.front().x(), diff_line.front().y(), 0.0};
                            concord::Point end_point{diff_line.back().x(), diff_line.back().y(), 0.0};

                            auto cut_swath =
                                std::make_shared<Swath>(create_swath(start_point, end_point, swath->type, ""));
                            result.push_back(cut_swath);
                        }
                    }

                    // Add connection swaths around obstacles
                    if (!differences.empty()) {
                        // Calculate connection points around the obstacle
                        for (size_t i = 0; i < differences.size() - 1; ++i) {
                            const auto &current_end = differences[i].back();
                            const auto &next_start = differences[i + 1].front();

                            concord::Point connection_start{current_end.x(), current_end.y(), 0.0};
                            concord::Point connection_end{next_start.x(), next_start.y(), 0.0};

                            auto connection_swath = std::make_shared<Swath>(
                                create_swath(connection_start, connection_end, SwathType::Around, ""));
                            result.push_back(connection_swath);
                        }
                    }
                } catch (const std::exception &e) {
                    // If difference operation fails, fall back to simple approach
                    // Just skip the problematic part for now
                    continue;
                }
            }

            return result;
        }
    };

} // namespace farmtrax
