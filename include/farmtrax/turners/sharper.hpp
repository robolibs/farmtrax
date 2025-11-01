#pragma once

#include <algorithm>
#include <cmath>
#include <concord/concord.hpp>
#include <vector>

namespace farmtrax {
    namespace turners {

        /**
         * @brief Sharper - Custom sharp turn planner for agricultural vehicles
         *
         * Creates safe turning patterns for vehicles that need to make sharp turns
         * while respecting machine dimensions. Generates waypoints at safe distances
         * (0.5x, 1x, 1.5x machine length) and connects them with proper arcs.
         *
         * Inspired by agricultural headland turning patterns:
         * - Three-point turn (switch-back)
         * - Bulb turn
         * - Fishtail turn
         */
        class Sharper {
          public:
            struct SharpTurnPath {
                std::vector<concord::Pose> waypoints;
                std::vector<std::string> segment_types; // "approach", "arc", "exit", etc.
                double total_length;
                std::string pattern_name; // "three_point", "bulb", "fishtail"
            };

            Sharper(double min_turning_radius, double machine_length, double machine_width = 0.0);

            /**
             * @brief Generate a sharp turn path at the same point
             * Creates overlapping lines from the turning point in both directions
             *
             * @param turning_point The point where the turn happens
             * @param old_heading Initial heading direction (radians)
             * @param new_heading Target heading direction (radians)
             * @param pattern Type of turn: "three_point", "bulb", "fishtail", "auto"
             * @return SharpTurnPath with waypoints and metadata
             */
            SharpTurnPath plan_sharp_turn_at_point(const concord::Point &turning_point, double old_heading,
                                                   double new_heading, const std::string &pattern = "auto") const;

            /**
             * @brief Generate a sharp turn path between two poses
             *
             * @param start Starting pose
             * @param end Ending pose
             * @param turn_angle Angle between start and end directions (radians)
             * @param pattern Type of turn: "three_point", "bulb", "fishtail", "auto"
             * @return SharpTurnPath with waypoints and metadata
             */
            SharpTurnPath plan_sharp_turn(const concord::Pose &start, const concord::Pose &end,
                                          const std::string &pattern = "auto") const;

            /**
             * @brief Generate a three-point (switch-back) turn
             * Used when machine needs to change direction at the same point
             * Pattern: forward along old heading -> reverse with turn -> forward along new heading
             */
            SharpTurnPath generate_three_point_turn(const concord::Pose &start, const concord::Pose &end,
                                                    double turn_angle) const;

            /**
             * @brief Generate a bulb turn
             * A wider turn that looks like a light bulb shape
             */
            SharpTurnPath generate_bulb_turn(const concord::Pose &start, const concord::Pose &end,
                                             double turn_angle) const;

            /**
             * @brief Generate a fishtail turn
             * An S-shaped turn that extends beyond the corner
             */
            SharpTurnPath generate_fishtail_turn(const concord::Pose &start, const concord::Pose &end,
                                                 double turn_angle) const;

            /**
             * @brief Create overlapping lines for sharp turn visualization
             * Returns the two line segments that define the turn geometry
             */
            struct TurnGeometry {
                concord::Point forward_end; // End of forward line (old heading)
                concord::Point reverse_end; // End of reverse line (new heading)
                concord::Point turn_center; // Center point of turn
                double turn_angle;          // Angle between headings
            };

            TurnGeometry calculate_turn_geometry(const concord::Pose &start, const concord::Pose &end) const;

          private:
            double radius_;
            double machine_length_;
            double machine_width_;

            double calculate_turn_angle(const concord::Pose &start, const concord::Pose &end) const;

            double normalize_angle(double angle) const;

            void calculate_path_length(SharpTurnPath &path) const;

            SharpTurnPath select_best_pattern(const concord::Pose &start, const concord::Pose &end,
                                              double turn_angle) const;
        };

    } // namespace turners
} // namespace farmtrax
