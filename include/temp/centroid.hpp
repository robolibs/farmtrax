#pragma once

#include <cmath>
#include <cstddef>

#include <datapod/datapod.hpp>

namespace temp {

    /**
     * @brief Calculate the centroid of a polygon
     *
     * Uses the formula for centroid of a polygon:
     * Cx = (1/6A) * sum((xi + xi+1) * (xi*yi+1 - xi+1*yi))
     * Cy = (1/6A) * sum((yi + yi+1) * (xi*yi+1 - xi+1*yi))
     *
     * @param polygon The polygon to calculate centroid for
     * @return The centroid point
     */
    inline datapod::Point centroid(const datapod::Polygon &polygon) {
        if (polygon.vertices.empty()) {
            return datapod::Point{0.0, 0.0, 0.0};
        }

        if (polygon.vertices.size() == 1) {
            return polygon.vertices[0];
        }

        if (polygon.vertices.size() == 2) {
            return datapod::Point{(polygon.vertices[0].x + polygon.vertices[1].x) / 2.0,
                                  (polygon.vertices[0].y + polygon.vertices[1].y) / 2.0,
                                  (polygon.vertices[0].z + polygon.vertices[1].z) / 2.0};
        }

        double signed_area = 0.0;
        double cx = 0.0;
        double cy = 0.0;

        const auto &verts = polygon.vertices;
        std::size_t n = verts.size();

        for (std::size_t i = 0; i < n; ++i) {
            std::size_t j = (i + 1) % n;
            double cross = verts[i].x * verts[j].y - verts[j].x * verts[i].y;
            signed_area += cross;
            cx += (verts[i].x + verts[j].x) * cross;
            cy += (verts[i].y + verts[j].y) * cross;
        }

        signed_area *= 0.5;

        if (std::abs(signed_area) < 1e-10) {
            // Degenerate polygon - return average of vertices
            double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
            for (const auto &v : verts) {
                sum_x += v.x;
                sum_y += v.y;
                sum_z += v.z;
            }
            return datapod::Point{sum_x / static_cast<double>(n), sum_y / static_cast<double>(n),
                                  sum_z / static_cast<double>(n)};
        }

        double factor = 1.0 / (6.0 * signed_area);
        cx *= factor;
        cy *= factor;

        // Average z coordinate
        double sum_z = 0.0;
        for (const auto &v : verts) {
            sum_z += v.z;
        }

        return datapod::Point{cx, cy, sum_z / static_cast<double>(n)};
    }

    /**
     * @brief Calculate the centroid of a ring
     */
    inline datapod::Point centroid(const datapod::Ring &ring) {
        datapod::Polygon poly;
        poly.vertices = ring.points;
        return centroid(poly);
    }

    /**
     * @brief Calculate the centroid of a linestring (midpoint of path)
     */
    inline datapod::Point centroid(const datapod::Linestring &line) {
        if (line.points.empty()) {
            return datapod::Point{0.0, 0.0, 0.0};
        }

        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        for (const auto &p : line.points) {
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
        }

        std::size_t n = line.points.size();
        return datapod::Point{sum_x / static_cast<double>(n), sum_y / static_cast<double>(n),
                              sum_z / static_cast<double>(n)};
    }

} // namespace temp
