#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <datapod/datapod.hpp>

namespace temp {

    /**
     * @brief Normalize a 2D vector
     */
    inline void normalize_2d(double &x, double &y) {
        double len = std::sqrt(x * x + y * y);
        if (len > 1e-10) {
            x /= len;
            y /= len;
        }
    }

    /**
     * @brief Compute the offset point at a vertex using miter join
     *
     * @param prev Previous vertex
     * @param curr Current vertex
     * @param next Next vertex
     * @param distance Offset distance (positive = outward, negative = inward)
     * @return Offset point
     */
    inline datapod::Point compute_miter_offset(const datapod::Point &prev, const datapod::Point &curr,
                                               const datapod::Point &next, double distance) {
        // Edge vectors
        double e1x = curr.x - prev.x;
        double e1y = curr.y - prev.y;
        double e2x = next.x - curr.x;
        double e2y = next.y - curr.y;

        // Normalize
        normalize_2d(e1x, e1y);
        normalize_2d(e2x, e2y);

        // Outward normals (perpendicular, pointing left of edge direction)
        double n1x = -e1y;
        double n1y = e1x;
        double n2x = -e2y;
        double n2y = e2x;

        // Bisector direction
        double bx = n1x + n2x;
        double by = n1y + n2y;
        double blen = std::sqrt(bx * bx + by * by);

        if (blen < 1e-10) {
            // Edges are parallel and opposite - use normal of first edge
            return datapod::Point{curr.x + n1x * distance, curr.y + n1y * distance, curr.z};
        }

        bx /= blen;
        by /= blen;

        // Miter length = distance / cos(half_angle)
        // cos(half_angle) = dot(n1, bisector)
        double cos_half = n1x * bx + n1y * by;

        // Limit miter to avoid spikes at sharp angles
        const double MITER_LIMIT = 2.0;
        double miter_length = distance;
        if (std::abs(cos_half) > 1e-10) {
            miter_length = distance / cos_half;
            // Clamp miter length
            if (std::abs(miter_length) > std::abs(distance) * MITER_LIMIT) {
                miter_length = distance * MITER_LIMIT * (miter_length > 0 ? 1.0 : -1.0);
            }
        }

        return datapod::Point{curr.x + bx * miter_length, curr.y + by * miter_length, curr.z};
    }

    /**
     * @brief Buffer (offset) a polygon by a given distance
     *
     * Positive distance expands the polygon outward.
     * Negative distance shrinks the polygon inward.
     *
     * Uses miter joins at vertices with a miter limit to handle sharp corners.
     *
     * @param polygon The polygon to buffer
     * @param distance The buffer distance (positive = expand, negative = shrink)
     * @return Buffered polygon
     */
    inline datapod::Polygon buffer_polygon(const datapod::Polygon &polygon, double distance) {
        datapod::Polygon result;

        if (polygon.vertices.size() < 3) {
            return result;
        }

        const auto &verts = polygon.vertices;
        std::size_t n = verts.size();

        result.vertices.reserve(n);

        for (std::size_t i = 0; i < n; ++i) {
            std::size_t prev_idx = (i + n - 1) % n;
            std::size_t next_idx = (i + 1) % n;

            datapod::Point offset_pt = compute_miter_offset(verts[prev_idx], verts[i], verts[next_idx], distance);
            result.vertices.push_back(offset_pt);
        }

        // For negative buffer (shrinking), check if polygon collapsed or self-intersects
        // Simple check: if area changed sign or became too small, the polygon collapsed
        if (distance < 0) {
            double new_area = result.area();

            // If area is negative (winding reversed) or too small, polygon collapsed
            // Use absolute minimum area threshold, not relative to original
            if (new_area < 1.0) { // Less than 1 sq meter is considered collapsed
                return datapod::Polygon{};
            }
        }

        return result;
    }

    /**
     * @brief Buffer a polygon with arc joins at vertices (smoother result)
     *
     * This version adds arc segments at convex vertices for smoother results.
     *
     * @param polygon The polygon to buffer
     * @param distance The buffer distance
     * @param segments_per_corner Number of segments for arc at each corner
     * @return Buffered polygon
     */
    inline datapod::Polygon buffer_polygon_rounded(const datapod::Polygon &polygon, double distance,
                                                   int segments_per_corner = 4) {
        datapod::Polygon result;

        if (polygon.vertices.size() < 3) {
            return result;
        }

        const auto &verts = polygon.vertices;
        std::size_t n = verts.size();

        for (std::size_t i = 0; i < n; ++i) {
            std::size_t prev_idx = (i + n - 1) % n;
            std::size_t next_idx = (i + 1) % n;

            const auto &prev = verts[prev_idx];
            const auto &curr = verts[i];
            const auto &next = verts[next_idx];

            // Edge vectors
            double e1x = curr.x - prev.x;
            double e1y = curr.y - prev.y;
            double e2x = next.x - curr.x;
            double e2y = next.y - curr.y;

            normalize_2d(e1x, e1y);
            normalize_2d(e2x, e2y);

            // Outward normals
            double n1x = -e1y;
            double n1y = e1x;
            double n2x = -e2y;
            double n2y = e2x;

            // Check if this is a convex or concave corner
            double cross = e1x * e2y - e1y * e2x;
            bool is_convex = (distance > 0) ? (cross < 0) : (cross > 0);

            if (is_convex && std::abs(distance) > 1e-10) {
                // Add arc at convex corner
                double angle1 = std::atan2(n1y, n1x);
                double angle2 = std::atan2(n2y, n2x);

                // Ensure we go the short way around
                double delta = angle2 - angle1;
                if (delta > M_PI)
                    delta -= 2 * M_PI;
                if (delta < -M_PI)
                    delta += 2 * M_PI;

                for (int j = 0; j <= segments_per_corner; ++j) {
                    double t = static_cast<double>(j) / segments_per_corner;
                    double angle = angle1 + t * delta;
                    double nx = std::cos(angle);
                    double ny = std::sin(angle);
                    result.vertices.push_back(datapod::Point{curr.x + nx * distance, curr.y + ny * distance, curr.z});
                }
            } else {
                // Use miter join for concave corners
                datapod::Point offset_pt = compute_miter_offset(prev, curr, next, distance);
                result.vertices.push_back(offset_pt);
            }
        }

        // Check for collapsed polygon
        if (distance < 0) {
            double new_area = result.area();

            // If area is negative (winding reversed) or too small, polygon collapsed
            if (new_area < 1.0) { // Less than 1 sq meter is considered collapsed
                return datapod::Polygon{};
            }
        }

        return result;
    }

    /**
     * @brief Shrink a polygon inward by a given distance
     *
     * Convenience function that calls buffer_polygon with negative distance.
     *
     * @param polygon The polygon to shrink
     * @param distance The shrink distance (positive value)
     * @return Shrunk polygon
     */
    inline datapod::Polygon shrink_polygon(const datapod::Polygon &polygon, double distance) {
        return buffer_polygon(polygon, -std::abs(distance));
    }

    /**
     * @brief Expand a polygon outward by a given distance
     *
     * Convenience function that calls buffer_polygon with positive distance.
     *
     * @param polygon The polygon to expand
     * @param distance The expand distance (positive value)
     * @return Expanded polygon
     */
    inline datapod::Polygon expand_polygon(const datapod::Polygon &polygon, double distance) {
        return buffer_polygon(polygon, std::abs(distance));
    }

} // namespace temp
