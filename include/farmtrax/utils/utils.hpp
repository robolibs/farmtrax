#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <datapod/datapod.hpp>

#include <temp/geometry.hpp>

namespace farmtrax {

    namespace utils {

        /**
         * @brief Convert a float value [0, 1] to a byte [0, 255]
         */
        inline uint8_t float_to_byte(float v, float min = 0.0f, float max = 255.0f) {
            v = std::clamp(v, 0.0f, 1.0f);
            float scaled = v * 255.0f;
            float clamped = std::clamp(scaled, min, max);
            return static_cast<uint8_t>(std::round(clamped));
        }

        /**
         * @brief Calculate distance from a point to a line segment
         *
         * @param point The point
         * @param line_start Start of the line segment
         * @param line_end End of the line segment
         * @return Distance from point to line segment
         */
        inline double point_to_line_distance(const datapod::Point &point, const datapod::Point &line_start,
                                             const datapod::Point &line_end) {
            datapod::Segment seg{line_start, line_end};
            return seg.distance_to(point);
        }

        /**
         * @brief Calculate angle between two vectors represented as Points (relative to origin)
         *
         * @param v1 First vector
         * @param v2 Second vector
         * @return Angle in radians
         */
        inline double angle_between(const datapod::Point &v1, const datapod::Point &v2) {
            // Calculate magnitudes
            double v1_mag = std::sqrt(v1.x * v1.x + v1.y * v1.y);
            double v2_mag = std::sqrt(v2.x * v2.x + v2.y * v2.y);

            // Prevent division by zero
            if (v1_mag < 1e-10 || v2_mag < 1e-10) {
                return 0.0; // One of the vectors is zero-length
            }

            // Calculate dot product and normalize
            double dot_product = (v1.x * v2.x + v1.y * v2.y) / (v1_mag * v2_mag);

            // Ensure dot product is within valid range for acos
            dot_product = std::clamp(dot_product, -1.0, 1.0);

            // Return angle in radians
            return std::acos(dot_product);
        }

        /**
         * @brief Check if three points are collinear
         *
         * @param p1 First point
         * @param p2 Second point
         * @param p3 Third point
         * @param epsilon Tolerance for collinearity check
         * @return true if points are collinear
         */
        inline bool are_colinear(const datapod::Point &p1, const datapod::Point &p2, const datapod::Point &p3,
                                 double epsilon = 1e-10) {
            // Calculate signed area of triangle formed by three points
            double area = (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y)) * 0.5;
            return std::abs(area) < epsilon;
        }

        /**
         * @brief Remove collinear points from a polygon
         *
         * @param polygon The polygon to simplify
         * @param epsilon Tolerance for collinearity check
         * @return Simplified polygon
         */
        inline datapod::Polygon remove_colinear_points(const datapod::Polygon &polygon, double epsilon = 0.01) {
            datapod::Polygon result;
            const auto &pts = polygon.vertices;

            if (pts.size() < 4) {
                return polygon;
            }

            // Check if polygon is closed (first == last)
            bool is_closed = false;
            if (pts.size() > 1) {
                double dx = pts.front().x - pts.back().x;
                double dy = pts.front().y - pts.back().y;
                is_closed = (dx * dx + dy * dy) < 1e-10;
            }

            std::size_t n = is_closed ? pts.size() - 1 : pts.size();

            for (std::size_t i = 0; i < n; ++i) {
                const auto &prev = pts[(i + n - 1) % n];
                const auto &curr = pts[i];
                const auto &next = pts[(i + 1) % n];

                if (!are_colinear(prev, curr, next, epsilon)) {
                    result.vertices.push_back(curr);
                }
            }

            // Close the polygon if it was closed
            if (is_closed && !result.vertices.empty()) {
                result.vertices.push_back(result.vertices.front());
            }

            return result;
        }

        /**
         * @brief Check if two points are approximately equal
         *
         * @param p1 First point
         * @param p2 Second point
         * @param epsilon Tolerance
         * @return true if points are approximately equal
         */
        inline bool points_equal(const datapod::Point &p1, const datapod::Point &p2, double epsilon = 1e-10) {
            double dx = p1.x - p2.x;
            double dy = p1.y - p2.y;
            double dz = p1.z - p2.z;
            return (dx * dx + dy * dy + dz * dz) < epsilon * epsilon;
        }

        /**
         * @brief Ensure polygon is closed (first point == last point)
         *
         * @param polygon The polygon to close
         * @return Closed polygon
         */
        inline datapod::Polygon close_polygon(const datapod::Polygon &polygon) {
            if (polygon.vertices.empty()) {
                return polygon;
            }

            datapod::Polygon result = polygon;

            if (!points_equal(result.vertices.front(), result.vertices.back())) {
                result.vertices.push_back(result.vertices.front());
            }

            return result;
        }

        /**
         * @brief Correct polygon winding order (ensure CCW for exterior rings)
         *
         * @param polygon The polygon to correct
         * @return Corrected polygon
         */
        inline datapod::Polygon correct_polygon(const datapod::Polygon &polygon) {
            return temp::correct_polygon(polygon);
        }

        /**
         * @brief Calculate the heading angle from one point to another
         *
         * @param from Start point
         * @param to End point
         * @return Heading angle in radians (0 = east, pi/2 = north)
         */
        inline double heading_between(const datapod::Point &from, const datapod::Point &to) {
            return std::atan2(to.y - from.y, to.x - from.x);
        }

        /**
         * @brief Normalize an angle to [-pi, pi]
         *
         * @param angle Angle in radians
         * @return Normalized angle
         */
        inline double normalize_angle(double angle) {
            while (angle > M_PI)
                angle -= 2.0 * M_PI;
            while (angle < -M_PI)
                angle += 2.0 * M_PI;
            return angle;
        }

        /**
         * @brief Calculate the shortest angular difference between two angles
         *
         * @param from Start angle in radians
         * @param to End angle in radians
         * @return Angular difference in radians [-pi, pi]
         */
        inline double angle_difference(double from, double to) { return normalize_angle(to - from); }

    } // namespace utils

} // namespace farmtrax
