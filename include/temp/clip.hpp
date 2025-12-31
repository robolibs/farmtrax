#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <datapod/datapod.hpp>

namespace temp {

    /**
     * @brief Check if a point is inside a polygon using ray casting
     *
     * @param point The point to check
     * @param polygon The polygon
     * @return true if point is inside polygon
     */
    inline bool point_in_polygon(const datapod::Point &point, const datapod::Polygon &polygon) {
        return polygon.contains(point);
    }

    /**
     * @brief Compute intersection point of two line segments
     *
     * @param p1 Start of first segment
     * @param p2 End of first segment
     * @param p3 Start of second segment
     * @param p4 End of second segment
     * @param intersection Output intersection point
     * @return true if segments intersect, false otherwise
     */
    inline bool segment_intersection(const datapod::Point &p1, const datapod::Point &p2, const datapod::Point &p3,
                                     const datapod::Point &p4, datapod::Point &intersection) {
        double d1x = p2.x - p1.x;
        double d1y = p2.y - p1.y;
        double d2x = p4.x - p3.x;
        double d2y = p4.y - p3.y;

        double cross = d1x * d2y - d1y * d2x;

        // Parallel lines
        if (std::abs(cross) < 1e-10) {
            return false;
        }

        double dx = p3.x - p1.x;
        double dy = p3.y - p1.y;

        double t = (dx * d2y - dy * d2x) / cross;
        double u = (dx * d1y - dy * d1x) / cross;

        // Check if intersection is within both segments
        if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
            intersection.x = p1.x + t * d1x;
            intersection.y = p1.y + t * d1y;
            intersection.z = p1.z + t * (p2.z - p1.z);
            return true;
        }

        return false;
    }

    /**
     * @brief Compute intersection point of a line (infinite) with a segment
     *
     * @param line_origin Point on the line
     * @param line_dir Direction of the line
     * @param seg_start Start of segment
     * @param seg_end End of segment
     * @param intersection Output intersection point
     * @param t Output parameter along line
     * @return true if line intersects segment
     */
    inline bool line_segment_intersection(const datapod::Point &line_origin, const datapod::Point &line_dir,
                                          const datapod::Point &seg_start, const datapod::Point &seg_end,
                                          datapod::Point &intersection, double &t) {
        double d2x = seg_end.x - seg_start.x;
        double d2y = seg_end.y - seg_start.y;

        double cross = line_dir.x * d2y - line_dir.y * d2x;

        // Parallel
        if (std::abs(cross) < 1e-10) {
            return false;
        }

        double dx = seg_start.x - line_origin.x;
        double dy = seg_start.y - line_origin.y;

        t = (dx * d2y - dy * d2x) / cross;
        double u = (dx * line_dir.y - dy * line_dir.x) / cross;

        // Check if intersection is within segment
        if (u >= 0.0 && u <= 1.0) {
            intersection.x = line_origin.x + t * line_dir.x;
            intersection.y = line_origin.y + t * line_dir.y;
            intersection.z = line_origin.z + t * line_dir.z;
            return true;
        }

        return false;
    }

    /**
     * @brief Check if a segment intersects a polygon
     *
     * @param segment The segment to check
     * @param polygon The polygon
     * @return true if segment intersects polygon boundary or is inside
     */
    inline bool intersects_segment_polygon(const datapod::Segment &segment, const datapod::Polygon &polygon) {
        // Check if either endpoint is inside
        if (polygon.contains(segment.start) || polygon.contains(segment.end)) {
            return true;
        }

        // Check for intersection with polygon edges
        const auto &verts = polygon.vertices;
        std::size_t n = verts.size();

        for (std::size_t i = 0; i < n; ++i) {
            std::size_t j = (i + 1) % n;
            datapod::Point intersection;
            if (segment_intersection(segment.start, segment.end, verts[i], verts[j], intersection)) {
                return true;
            }
        }

        return false;
    }

    /**
     * @brief Clip a segment to a polygon boundary
     *
     * Returns the portions of the segment that are inside the polygon.
     * Uses Sutherland-Hodgman style clipping.
     *
     * @param segment The segment to clip
     * @param polygon The polygon to clip against
     * @return Vector of segments that are inside the polygon
     */
    inline std::vector<datapod::Segment> clip_segment_to_polygon(const datapod::Segment &segment,
                                                                 const datapod::Polygon &polygon) {
        std::vector<datapod::Segment> result;

        if (polygon.vertices.size() < 3) {
            return result;
        }

        // Collect all intersection points with polygon edges
        struct Intersection {
            double t; // Parameter along segment [0, 1]
            datapod::Point point;
            bool entering; // true if entering polygon, false if exiting
        };

        std::vector<Intersection> intersections;

        const auto &verts = polygon.vertices;
        std::size_t n = verts.size();

        datapod::Point seg_dir{segment.end.x - segment.start.x, segment.end.y - segment.start.y,
                               segment.end.z - segment.start.z};

        for (std::size_t i = 0; i < n; ++i) {
            std::size_t j = (i + 1) % n;

            datapod::Point intersection;
            double t;

            // Check intersection with this edge
            double d2x = verts[j].x - verts[i].x;
            double d2y = verts[j].y - verts[i].y;

            double cross = seg_dir.x * d2y - seg_dir.y * d2x;

            if (std::abs(cross) < 1e-10) {
                continue; // Parallel
            }

            double dx = verts[i].x - segment.start.x;
            double dy = verts[i].y - segment.start.y;

            t = (dx * d2y - dy * d2x) / cross;
            double u = (dx * seg_dir.y - dy * seg_dir.x) / cross;

            if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
                intersection.x = segment.start.x + t * seg_dir.x;
                intersection.y = segment.start.y + t * seg_dir.y;
                intersection.z = segment.start.z + t * seg_dir.z;

                // Determine if entering or exiting based on edge normal
                double edge_nx = d2y;
                double edge_ny = -d2x;
                bool entering = (seg_dir.x * edge_nx + seg_dir.y * edge_ny) < 0;

                intersections.push_back({t, intersection, entering});
            }
        }

        // Sort intersections by parameter t
        std::sort(intersections.begin(), intersections.end(),
                  [](const Intersection &a, const Intersection &b) { return a.t < b.t; });

        // Build result segments
        bool start_inside = polygon.contains(segment.start);
        bool currently_inside = start_inside;
        datapod::Point current_start = segment.start;

        for (const auto &isect : intersections) {
            if (currently_inside) {
                // We're inside, this intersection exits
                result.push_back(datapod::Segment{current_start, isect.point});
                currently_inside = false;
            } else {
                // We're outside, this intersection enters
                current_start = isect.point;
                currently_inside = true;
            }
        }

        // Handle final segment if we end inside
        if (currently_inside) {
            result.push_back(datapod::Segment{current_start, segment.end});
        }

        return result;
    }

    /**
     * @brief Clip a linestring to a polygon boundary
     *
     * @param linestring The linestring to clip
     * @param polygon The polygon to clip against
     * @return Vector of segments that are inside the polygon
     */
    inline std::vector<datapod::Segment> clip_linestring_to_polygon(const datapod::Linestring &linestring,
                                                                    const datapod::Polygon &polygon) {
        std::vector<datapod::Segment> result;

        if (linestring.points.size() < 2) {
            return result;
        }

        for (std::size_t i = 0; i < linestring.points.size() - 1; ++i) {
            datapod::Segment seg{linestring.points[i], linestring.points[i + 1]};
            auto clipped = clip_segment_to_polygon(seg, polygon);
            result.insert(result.end(), clipped.begin(), clipped.end());
        }

        return result;
    }

} // namespace temp
