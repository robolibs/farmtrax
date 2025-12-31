#pragma once

#include "farmtrax/field.hpp"
#include "farmtrax/utils/utils.hpp"

#include <datapod/datapod.hpp>
#include <temp/geometry.hpp>

#include <iostream>
#include <memory>
#include <vector>

namespace farmtrax {

    /**
     * @brief Obstacle avoidance for swath planning
     *
     * Takes obstacles as polygons and modifies swaths to avoid them.
     * Supports inflation (buffer) of obstacles for safety margins.
     */
    class ObstacleAvoider {
        std::vector<datapod::Polygon> obstacles_;
        std::vector<datapod::Polygon> inflated_obstacles_;
        float inflation_distance_;
        datapod::Geo datum_;

      public:
        /**
         * @brief Constructor takes array of polygons as obstacles
         */
        inline explicit ObstacleAvoider(const std::vector<datapod::Polygon> &obstacles,
                                        const datapod::Geo &datum = datapod::Geo{})
            : obstacles_(obstacles), inflation_distance_(0.0f), datum_(datum) {}

        /**
         * @brief Main avoidance method
         *
         * @param input_swaths Swaths to process
         * @param inflation_distance Distance to inflate obstacles
         * @return Processed swaths that avoid obstacles
         */
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

        /**
         * @brief Overload for non-const swaths
         */
        inline std::vector<std::shared_ptr<Swath>> avoid(const std::vector<std::shared_ptr<Swath>> &input_swaths,
                                                         float inflation_distance) {
            std::vector<std::shared_ptr<const Swath>> const_swaths;
            const_swaths.reserve(input_swaths.size());
            for (const auto &swath : input_swaths) {
                const_swaths.push_back(swath);
            }
            return avoid(const_swaths, inflation_distance);
        }

        /**
         * @brief Get original obstacles
         */
        inline const std::vector<datapod::Polygon> &get_obstacles() const { return obstacles_; }

        /**
         * @brief Get inflated obstacles
         */
        inline const std::vector<datapod::Polygon> &get_inflated_obstacles() const { return inflated_obstacles_; }

      private:
        /**
         * @brief Inflate all obstacles by the given distance
         */
        inline void inflate_obstacles() {
            inflated_obstacles_.clear();
            inflated_obstacles_.reserve(obstacles_.size());

            for (const auto &obstacle : obstacles_) {
                datapod::Polygon inflated = temp::expand_polygon(obstacle, inflation_distance_);
                if (!inflated.vertices.empty() && inflated.vertices.size() >= 3) {
                    inflated_obstacles_.push_back(inflated);
                }
            }
        }

        /**
         * @brief Process a single swath against all obstacles
         */
        inline std::vector<std::shared_ptr<Swath>> process_swath(const std::shared_ptr<const Swath> &swath) {
            std::vector<std::shared_ptr<Swath>> result;

            if (!swath) {
                return result;
            }

            // Check if swath intersects with any obstacle
            bool intersects = false;
            for (const auto &obstacle : inflated_obstacles_) {
                if (temp::intersects_segment_polygon(swath->line, obstacle)) {
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

        /**
         * @brief Cut swath around obstacles using difference operation
         */
        inline std::vector<std::shared_ptr<Swath>>
        cut_swath_around_obstacles(const std::shared_ptr<const Swath> &swath) {
            std::vector<std::shared_ptr<Swath>> result;

            if (!swath) {
                return result;
            }

            // Start with the original segment
            std::vector<datapod::Segment> current_segments;
            current_segments.push_back(swath->line);

            // For each obstacle, compute the difference
            for (const auto &obstacle : inflated_obstacles_) {
                std::vector<datapod::Segment> new_segments;

                for (const auto &seg : current_segments) {
                    // Use difference operation from temp/boolean.hpp
                    auto differences = temp::difference_segment_polygon(seg, obstacle);

                    if (differences.empty()) {
                        // Segment is entirely inside obstacle, skip it
                        continue;
                    }

                    new_segments.insert(new_segments.end(), differences.begin(), differences.end());
                }

                current_segments = std::move(new_segments);
            }

            // Convert remaining segments to Swath objects
            for (size_t i = 0; i < current_segments.size(); ++i) {
                const auto &seg = current_segments[i];

                // Skip very short segments
                if (seg.length() < 0.1) {
                    continue;
                }

                auto cut_swath = std::make_shared<Swath>(create_swath(seg.start, seg.end, swath->type, ""));
                result.push_back(cut_swath);

                // Add connection swaths between segments
                if (i < current_segments.size() - 1) {
                    const auto &next_seg = current_segments[i + 1];

                    // Only add connection if there's a gap
                    double gap = seg.end.distance_to(next_seg.start);
                    if (gap > 0.1) {
                        auto connection_swath =
                            std::make_shared<Swath>(create_swath(seg.end, next_seg.start, SwathType::Around, ""));
                        result.push_back(connection_swath);
                    }
                }
            }

            return result;
        }
    };

} // namespace farmtrax
