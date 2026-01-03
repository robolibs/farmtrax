#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <datapod/datapod.hpp>

#include "clip.hpp"

namespace temp {

    /**
     * @brief Compute the difference of a segment and a polygon
     *
     * Returns the portions of the segment that are OUTSIDE the polygon.
     * This is the complement of clip_segment_to_polygon.
     *
     * @param segment The segment
     * @param polygon The polygon to subtract
     * @return Vector of segments that are outside the polygon
     */
    inline std::vector<datapod::Segment> difference_segment_polygon(const datapod::Segment &segment,
                                                                    const datapod::Polygon &polygon) {
        std::vector<datapod::Segment> result;

        if (polygon.vertices.size() < 3) {
            // No polygon to subtract, return original segment
            result.push_back(segment);
            return result;
        }

        // Collect all intersection points with polygon edges
        struct Intersection {
            double t; // Parameter along segment [0, 1]
            datapod::Point point;
        };

        std::vector<Intersection> intersections;

        const auto &verts = polygon.vertices;
        std::size_t n = verts.size();

        datapod::Point seg_dir{segment.end.x - segment.start.x, segment.end.y - segment.start.y,
                               segment.end.z - segment.start.z};

        for (std::size_t i = 0; i < n; ++i) {
            std::size_t j = (i + 1) % n;

            double d2x = verts[j].x - verts[i].x;
            double d2y = verts[j].y - verts[i].y;

            double cross = seg_dir.x * d2y - seg_dir.y * d2x;

            if (std::abs(cross) < 1e-10) {
                continue; // Parallel
            }

            double dx = verts[i].x - segment.start.x;
            double dy = verts[i].y - segment.start.y;

            double t = (dx * d2y - dy * d2x) / cross;
            double u = (dx * seg_dir.y - dy * seg_dir.x) / cross;

            if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
                datapod::Point isect_pt;
                isect_pt.x = segment.start.x + t * seg_dir.x;
                isect_pt.y = segment.start.y + t * seg_dir.y;
                isect_pt.z = segment.start.z + t * seg_dir.z;

                intersections.push_back({t, isect_pt});
            }
        }

        // Add endpoints
        intersections.push_back({0.0, segment.start});
        intersections.push_back({1.0, segment.end});

        // Sort intersections by parameter t
        std::sort(intersections.begin(), intersections.end(),
                  [](const Intersection &a, const Intersection &b) { return a.t < b.t; });

        // Remove duplicates
        auto last =
            std::unique(intersections.begin(), intersections.end(),
                        [](const Intersection &a, const Intersection &b) { return std::abs(a.t - b.t) < 1e-10; });
        intersections.erase(last, intersections.end());

        // Build result segments - keep segments that are OUTSIDE the polygon
        for (std::size_t i = 0; i < intersections.size() - 1; ++i) {
            datapod::Point midpoint{(intersections[i].point.x + intersections[i + 1].point.x) / 2.0,
                                    (intersections[i].point.y + intersections[i + 1].point.y) / 2.0,
                                    (intersections[i].point.z + intersections[i + 1].point.z) / 2.0};

            // If midpoint is OUTSIDE polygon, keep this segment
            if (!polygon.contains(midpoint)) {
                // Skip very short segments
                double dx = intersections[i + 1].point.x - intersections[i].point.x;
                double dy = intersections[i + 1].point.y - intersections[i].point.y;
                if (dx * dx + dy * dy > 1e-10) {
                    result.push_back(datapod::Segment{intersections[i].point, intersections[i + 1].point});
                }
            }
        }

        return result;
    }

    /**
     * @brief Compute the difference of a linestring and a polygon
     *
     * @param linestring The linestring
     * @param polygon The polygon to subtract
     * @return Vector of segments that are outside the polygon
     */
    inline std::vector<datapod::Segment> difference_linestring_polygon(const datapod::Linestring &linestring,
                                                                       const datapod::Polygon &polygon) {
        std::vector<datapod::Segment> result;

        if (linestring.points.size() < 2) {
            return result;
        }

        for (std::size_t i = 0; i < linestring.points.size() - 1; ++i) {
            datapod::Segment seg{linestring.points[i], linestring.points[i + 1]};
            auto diff = difference_segment_polygon(seg, polygon);
            result.insert(result.end(), diff.begin(), diff.end());
        }

        return result;
    }

    /**
     * @brief Compute the difference of a segment and multiple polygons
     *
     * @param segment The segment
     * @param polygons The polygons to subtract
     * @return Vector of segments that are outside all polygons
     */
    inline std::vector<datapod::Segment> difference_segment_polygons(const datapod::Segment &segment,
                                                                     const std::vector<datapod::Polygon> &polygons) {
        std::vector<datapod::Segment> current;
        current.push_back(segment);

        for (const auto &polygon : polygons) {
            std::vector<datapod::Segment> next;
            for (const auto &seg : current) {
                auto diff = difference_segment_polygon(seg, polygon);
                next.insert(next.end(), diff.begin(), diff.end());
            }
            current = std::move(next);
        }

        return current;
    }

} // namespace temp
