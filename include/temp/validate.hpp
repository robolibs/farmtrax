#pragma once

#include <cmath>
#include <cstddef>

#include <datapod/datapod.hpp>

namespace temp {

    /**
     * @brief Calculate the signed area of a polygon
     *
     * Positive area means counter-clockwise winding (exterior ring)
     * Negative area means clockwise winding (interior ring/hole)
     *
     * @param polygon The polygon to calculate signed area for
     * @return Signed area (positive = CCW, negative = CW)
     */
    inline double signed_area(const datapod::Polygon &polygon) {
        if (polygon.vertices.size() < 3) {
            return 0.0;
        }

        double area = 0.0;
        const auto &verts = polygon.vertices;
        std::size_t n = verts.size();

        for (std::size_t i = 0; i < n; ++i) {
            std::size_t j = (i + 1) % n;
            area += verts[i].x * verts[j].y;
            area -= verts[j].x * verts[i].y;
        }

        return area * 0.5;
    }

    /**
     * @brief Check if polygon has counter-clockwise winding order
     *
     * @param polygon The polygon to check
     * @return true if CCW (exterior ring convention), false if CW
     */
    inline bool is_ccw(const datapod::Polygon &polygon) { return signed_area(polygon) > 0.0; }

    /**
     * @brief Check if polygon has clockwise winding order
     *
     * @param polygon The polygon to check
     * @return true if CW (hole convention), false if CCW
     */
    inline bool is_cw(const datapod::Polygon &polygon) { return signed_area(polygon) < 0.0; }

    /**
     * @brief Check if a polygon is valid
     *
     * A polygon is valid if:
     * - It has at least 3 vertices
     * - It has non-zero area
     * - No self-intersections (simplified check - just checks for degenerate cases)
     *
     * @param polygon The polygon to validate
     * @return true if valid, false otherwise
     */
    inline bool is_valid_polygon(const datapod::Polygon &polygon) {
        // Must have at least 3 vertices
        if (polygon.vertices.size() < 3) {
            return false;
        }

        // Must have non-zero area
        double area = std::abs(signed_area(polygon));
        if (area < 1e-10) {
            return false;
        }

        // Check for duplicate consecutive vertices (excluding the closing vertex)
        const auto &verts = polygon.vertices;
        std::size_t n = verts.size();

        // Check if polygon is closed (first == last)
        bool is_closed = false;
        if (n >= 2) {
            double dx = verts[0].x - verts[n - 1].x;
            double dy = verts[0].y - verts[n - 1].y;
            is_closed = (dx * dx + dy * dy < 1e-20);
        }

        // Check for duplicate consecutive vertices (skip last->first if closed)
        std::size_t check_count = is_closed ? n - 1 : n;
        for (std::size_t i = 0; i < check_count; ++i) {
            std::size_t j = (i + 1) % n;
            double dx = verts[i].x - verts[j].x;
            double dy = verts[i].y - verts[j].y;
            if (dx * dx + dy * dy < 1e-20) {
                return false;
            }
        }

        return true;
    }

    /**
     * @brief Correct polygon winding order to counter-clockwise (exterior ring convention)
     *
     * This is the standard convention for exterior rings in most geometry libraries.
     *
     * @param polygon The polygon to correct
     * @return Polygon with CCW winding order
     */
    inline datapod::Polygon correct_polygon(const datapod::Polygon &polygon) {
        if (polygon.vertices.size() < 3) {
            return polygon;
        }

        // If already CCW, return as-is
        if (is_ccw(polygon)) {
            return polygon;
        }

        // Reverse the vertex order to make it CCW
        datapod::Polygon result;
        result.vertices.reserve(polygon.vertices.size());

        for (auto it = polygon.vertices.rbegin(); it != polygon.vertices.rend(); ++it) {
            result.vertices.push_back(*it);
        }

        return result;
    }

    /**
     * @brief Correct polygon winding order to clockwise (hole convention)
     *
     * @param polygon The polygon to correct
     * @return Polygon with CW winding order
     */
    inline datapod::Polygon correct_polygon_cw(const datapod::Polygon &polygon) {
        if (polygon.vertices.size() < 3) {
            return polygon;
        }

        // If already CW, return as-is
        if (is_cw(polygon)) {
            return polygon;
        }

        // Reverse the vertex order to make it CW
        datapod::Polygon result;
        result.vertices.reserve(polygon.vertices.size());

        for (auto it = polygon.vertices.rbegin(); it != polygon.vertices.rend(); ++it) {
            result.vertices.push_back(*it);
        }

        return result;
    }

    /**
     * @brief Remove duplicate consecutive vertices from a polygon
     *
     * @param polygon The polygon to clean
     * @param epsilon Tolerance for considering points equal
     * @return Cleaned polygon
     */
    inline datapod::Polygon remove_duplicate_vertices(const datapod::Polygon &polygon, double epsilon = 1e-10) {
        if (polygon.vertices.size() < 2) {
            return polygon;
        }

        datapod::Polygon result;
        result.vertices.reserve(polygon.vertices.size());

        const auto &verts = polygon.vertices;
        result.vertices.push_back(verts[0]);

        for (std::size_t i = 1; i < verts.size(); ++i) {
            const auto &prev = result.vertices.back();
            const auto &curr = verts[i];

            double dx = curr.x - prev.x;
            double dy = curr.y - prev.y;
            double dz = curr.z - prev.z;

            if (dx * dx + dy * dy + dz * dz > epsilon * epsilon) {
                result.vertices.push_back(curr);
            }
        }

        // Check if first and last are duplicates
        if (result.vertices.size() > 1) {
            const auto &first = result.vertices.front();
            const auto &last = result.vertices.back();
            double dx = last.x - first.x;
            double dy = last.y - first.y;
            double dz = last.z - first.z;

            if (dx * dx + dy * dy + dz * dz < epsilon * epsilon) {
                result.vertices.pop_back();
            }
        }

        return result;
    }

} // namespace temp
