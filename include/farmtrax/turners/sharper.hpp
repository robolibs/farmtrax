#pragma once

#include "dubins.hpp" // For Pose2D
#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
                std::vector<Pose2D> waypoints;
                std::vector<std::string> segment_types; // "approach", "arc", "exit", etc.
                double total_length;
                std::string pattern_name; // "three_point", "bulb", "fishtail"
            };

            inline Sharper(double min_turning_radius, double machine_length, double machine_width = 0.0)
                : radius_(min_turning_radius), machine_length_(machine_length),
                  machine_width_(machine_width > 0 ? machine_width : machine_length * 0.4) {}

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
            inline SharpTurnPath plan_sharp_turn_at_point(const datapod::Point &turning_point, double old_heading,
                                                          double new_heading,
                                                          const std::string &pattern = "auto") const {
                Pose2D start(turning_point, old_heading);
                Pose2D end(turning_point, new_heading);

                return plan_sharp_turn(start, end, pattern);
            }

            /**
             * @brief Generate a sharp turn path between two poses
             *
             * @param start Starting pose
             * @param end Ending pose
             * @param turn_angle Angle between start and end directions (radians)
             * @param pattern Type of turn: "three_point", "bulb", "fishtail", "auto"
             * @return SharpTurnPath with waypoints and metadata
             */
            inline SharpTurnPath plan_sharp_turn(const Pose2D &start, const Pose2D &end,
                                                 const std::string &pattern = "auto") const {
                double turn_angle = calculate_turn_angle(start, end);

                if (pattern == "auto") {
                    return select_best_pattern(start, end, turn_angle);
                } else if (pattern == "three_point") {
                    return generate_three_point_turn(start, end, turn_angle);
                } else if (pattern == "bulb") {
                    return generate_bulb_turn(start, end, turn_angle);
                } else if (pattern == "fishtail") {
                    return generate_fishtail_turn(start, end, turn_angle);
                }

                return SharpTurnPath();
            }

            /**
             * @brief Generate a three-point (switch-back) turn
             * Used when machine needs to change direction at the same point
             * Pattern: forward along old heading -> reverse with turn -> forward along new heading
             */
            inline SharpTurnPath generate_three_point_turn(const Pose2D &start, const Pose2D &end,
                                                           double turn_angle) const {
                SharpTurnPath path;
                path.pattern_name = "three_point";

                double forward_dist = machine_length_;
                double reverse_dist = machine_length_;

                path.waypoints.push_back(start);
                path.segment_types.push_back("start");

                Pose2D forward_point;
                forward_point.point.x = start.point.x + forward_dist * cos(start.yaw);
                forward_point.point.y = start.point.y + forward_dist * sin(start.yaw);
                forward_point.yaw = start.yaw;
                path.waypoints.push_back(forward_point);
                path.segment_types.push_back("forward_along_old_heading");

                Pose2D reverse_point;
                reverse_point.point.x = start.point.x - reverse_dist * cos(end.yaw);
                reverse_point.point.y = start.point.y - reverse_dist * sin(end.yaw);
                reverse_point.yaw = end.yaw;
                path.waypoints.push_back(reverse_point);
                path.segment_types.push_back("reverse_with_turn");

                path.waypoints.push_back(end);
                path.segment_types.push_back("forward_to_turning_point");

                calculate_path_length(path);
                return path;
            }

            /**
             * @brief Generate a bulb turn
             * A wider turn that looks like a light bulb shape
             */
            inline SharpTurnPath generate_bulb_turn(const Pose2D &start, const Pose2D &end, double turn_angle) const {
                SharpTurnPath path;
                path.pattern_name = "bulb";

                double bulb_radius = machine_length_ * 0.8;

                path.waypoints.push_back(start);
                path.segment_types.push_back("start");

                int num_bulb_points = 5;
                for (int i = 1; i <= num_bulb_points; ++i) {
                    double t = i / double(num_bulb_points + 1);
                    double current_angle = normalize_angle(start.yaw + turn_angle * t);

                    double bulb_factor = sin(M_PI * t) * bulb_radius;
                    double perpendicular_angle = current_angle + M_PI / 2;

                    Pose2D bulb_point;
                    bulb_point.point.x = start.point.x + radius_ * t * cos(current_angle);
                    bulb_point.point.y = start.point.y + radius_ * t * sin(current_angle);

                    bulb_point.point.x += bulb_factor * cos(perpendicular_angle);
                    bulb_point.point.y += bulb_factor * sin(perpendicular_angle);
                    bulb_point.yaw = current_angle;

                    path.waypoints.push_back(bulb_point);
                    path.segment_types.push_back("bulb_arc");
                }

                path.waypoints.push_back(end);
                path.segment_types.push_back("end");

                calculate_path_length(path);
                return path;
            }

            /**
             * @brief Generate a fishtail turn
             * An S-shaped turn that extends beyond the corner
             */
            inline SharpTurnPath generate_fishtail_turn(const Pose2D &start, const Pose2D &end,
                                                        double turn_angle) const {
                SharpTurnPath path;
                path.pattern_name = "fishtail";

                double extend_dist = machine_length_ * 1.2;

                path.waypoints.push_back(start);
                path.segment_types.push_back("start");

                Pose2D approach;
                approach.point.x = start.point.x + 0.5 * machine_length_ * cos(start.yaw);
                approach.point.y = start.point.y + 0.5 * machine_length_ * sin(start.yaw);
                approach.yaw = start.yaw;
                path.waypoints.push_back(approach);
                path.segment_types.push_back("approach");

                double tail_angle = normalize_angle(start.yaw - turn_angle * 0.4);
                Pose2D tail_point;
                tail_point.point.x = approach.point.x + extend_dist * cos(tail_angle);
                tail_point.point.y = approach.point.y + extend_dist * sin(tail_angle);
                tail_point.yaw = tail_angle;
                path.waypoints.push_back(tail_point);
                path.segment_types.push_back("tail_out");

                double transition_angle = normalize_angle(start.yaw + turn_angle * 0.7);
                Pose2D transition_point;
                transition_point.point.x = tail_point.point.x + machine_length_ * cos(transition_angle);
                transition_point.point.y = tail_point.point.y + machine_length_ * sin(transition_angle);
                transition_point.yaw = transition_angle;
                path.waypoints.push_back(transition_point);
                path.segment_types.push_back("transition");

                Pose2D final_approach;
                final_approach.point.x = end.point.x - 0.5 * machine_length_ * cos(end.yaw);
                final_approach.point.y = end.point.y - 0.5 * machine_length_ * sin(end.yaw);
                final_approach.yaw = end.yaw;
                path.waypoints.push_back(final_approach);
                path.segment_types.push_back("final_approach");

                path.waypoints.push_back(end);
                path.segment_types.push_back("end");

                calculate_path_length(path);
                return path;
            }

            /**
             * @brief Create overlapping lines for sharp turn visualization
             * Returns the two line segments that define the turn geometry
             */
            struct TurnGeometry {
                datapod::Point forward_end; // End of forward line (old heading)
                datapod::Point reverse_end; // End of reverse line (new heading)
                datapod::Point turn_center; // Center point of turn
                double turn_angle;          // Angle between headings
            };

            inline TurnGeometry calculate_turn_geometry(const Pose2D &start, const Pose2D &end) const {
                TurnGeometry geom;
                geom.turn_center = start.point;
                geom.turn_angle = calculate_turn_angle(start, end);

                // Forward line along old heading
                geom.forward_end.x = start.point.x + machine_length_ * cos(start.yaw);
                geom.forward_end.y = start.point.y + machine_length_ * sin(start.yaw);

                // Reverse line along new heading (backwards from center)
                geom.reverse_end.x = start.point.x - machine_length_ * cos(end.yaw);
                geom.reverse_end.y = start.point.y - machine_length_ * sin(end.yaw);

                return geom;
            }

          private:
            double radius_;
            double machine_length_;
            double machine_width_;

            inline double calculate_turn_angle(const Pose2D &start, const Pose2D &end) const {
                return normalize_angle(end.yaw - start.yaw);
            }

            inline double normalize_angle(double angle) const {
                while (angle > M_PI)
                    angle -= 2 * M_PI;
                while (angle < -M_PI)
                    angle += 2 * M_PI;
                return angle;
            }

            inline void calculate_path_length(SharpTurnPath &path) const {
                path.total_length = 0.0;
                for (size_t i = 1; i < path.waypoints.size(); ++i) {
                    double dx = path.waypoints[i].point.x - path.waypoints[i - 1].point.x;
                    double dy = path.waypoints[i].point.y - path.waypoints[i - 1].point.y;
                    path.total_length += sqrt(dx * dx + dy * dy);
                }
            }

            inline SharpTurnPath select_best_pattern(const Pose2D &start, const Pose2D &end, double turn_angle) const {
                double angle_deg = fabs(turn_angle) * 180.0 / M_PI;

                double dist = sqrt(pow(end.point.x - start.point.x, 2) + pow(end.point.y - start.point.y, 2));

                bool same_point_turn = (dist < 0.1 * machine_length_);

                if (same_point_turn) {
                    return generate_three_point_turn(start, end, turn_angle);
                }

                if (angle_deg > 120) {
                    return generate_three_point_turn(start, end, turn_angle);
                } else if (angle_deg > 60) {
                    return generate_bulb_turn(start, end, turn_angle);
                } else {
                    return generate_fishtail_turn(start, end, turn_angle);
                }
            }
        };

    } // namespace turners
} // namespace farmtrax
