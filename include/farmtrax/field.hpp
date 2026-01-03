#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <datapod/datapod.hpp>

#include <temp/geometry.hpp>

#include "farmtrax/utils/utils.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace farmtrax {

    /**
     * @brief Ring represents a closed polygon boundary (headland or field boundary)
     */
    struct Ring {
        datapod::Polygon polygon;
        std::string uuid;
        bool finished = false;
        datapod::AABB bounding_box;
    };

    /**
     * @brief Create a Ring from a polygon with optional UUID
     *
     * @param poly The polygon defining the ring boundary
     * @param uuid Optional UUID (generated if empty)
     * @return Ring structure with computed bounding box
     */
    inline Ring create_ring(const datapod::Polygon &poly, std::string uuid = "") {
        if (uuid.empty()) {
            uuid = temp::generate_uuid();
        }

        Ring ring;
        ring.polygon = poly;
        ring.uuid = uuid;
        ring.finished = false;

        // Ensure polygon is closed and correct
        if (!ring.polygon.vertices.empty()) {
            // Close polygon if needed
            if (ring.polygon.vertices.size() > 1) {
                const auto &first = ring.polygon.vertices.front();
                const auto &last = ring.polygon.vertices.back();
                double dx = first.x - last.x;
                double dy = first.y - last.y;
                if (dx * dx + dy * dy > 1e-10) {
                    ring.polygon.vertices.push_back(first);
                }
            }
            ring.polygon = temp::correct_polygon(ring.polygon);
        }

        ring.bounding_box = ring.polygon.get_aabb();
        return ring;
    }

    /**
     * @brief Type of swath/path segment
     */
    enum class SwathType {
        Swath,      ///< Regular working swath
        Connection, ///< Connection between swaths
        Around,     ///< Around obstacle
        Headland,   ///< Headland pass
    };

    /**
     * @brief Swath represents a working path segment
     */
    struct Swath {
        datapod::Segment line;
        std::string uuid;
        SwathType type = SwathType::Swath;
        bool finished = false;
        datapod::AABB bounding_box;

        // Additional fields for testing/compatibility
        int id = -1;
        double width = 0.0;
        datapod::Vector<datapod::Point> points;

        /**
         * @brief Get the head (start) point of the swath
         */
        datapod::Point getHead() const { return line.start; }

        /**
         * @brief Get the tail (end) point of the swath
         */
        datapod::Point getTail() const { return line.end; }

        /**
         * @brief Swap head and tail (reverse direction)
         */
        void swapDirection() {
            std::swap(line.start, line.end);
            // Bounding box doesn't change when swapping direction
        }

        /**
         * @brief Create a copy with swapped direction
         */
        Swath withSwappedDirection() const {
            Swath swapped = *this;
            swapped.swapDirection();
            return swapped;
        }
    };

    /**
     * @brief Create a Swath from start and end points
     *
     * @param start Start point of the swath
     * @param end End point of the swath
     * @param type Type of swath
     * @param uuid Optional UUID (generated if empty)
     * @return Swath structure with computed bounding box
     */
    inline Swath create_swath(const datapod::Point &start, const datapod::Point &end, SwathType type,
                              std::string uuid = "") {
        if (uuid.empty()) {
            uuid = temp::generate_uuid();
        }

        Swath swath;
        swath.line = datapod::Segment{start, end};
        swath.uuid = uuid;
        swath.type = type;
        swath.finished = false;

        // Compute bounding box
        swath.bounding_box =
            datapod::AABB{datapod::Point{std::min(start.x, end.x), std::min(start.y, end.y), std::min(start.z, end.z)},
                          datapod::Point{std::max(start.x, end.x), std::max(start.y, end.y), std::max(start.z, end.z)}};

        return swath;
    }

    /**
     * @brief Part represents a subdivided section of the field
     */
    struct Part {
        Ring boundary; ///< The boundary polygon of this subdivided part
        std::vector<Swath> swaths;
        std::vector<Ring> headlands;

        // R-trees for spatial indexing
        datapod::RTree<std::size_t> swath_rtree;
        datapod::RTree<std::size_t> headland_rtree;
        datapod::PointRTree<std::size_t> swath_endpoints_rtree;

        /**
         * @brief Rebuild all R-trees after geometry changes
         */
        void rebuild_rtrees() {
            // Clear existing trees
            swath_rtree.clear();
            headland_rtree.clear();
            swath_endpoints_rtree.clear();

            // Rebuild swath R-tree
            for (std::size_t i = 0; i < swaths.size(); ++i) {
                swath_rtree.insert(swaths[i].bounding_box, i);

                // Add start and end points to point R-tree
                swath_endpoints_rtree.insert(swaths[i].line.start, i * 2);   // Even indices for start points
                swath_endpoints_rtree.insert(swaths[i].line.end, i * 2 + 1); // Odd indices for end points
            }

            // Rebuild headland R-tree
            for (std::size_t i = 0; i < headlands.size(); ++i) {
                headland_rtree.insert(headlands[i].bounding_box, i);
            }
        }
    };

    /**
     * @brief Field represents an agricultural field with headlands and swaths
     *
     * The Field class manages field geometry, headland generation, and swath planning.
     * It supports field subdivision and spatial queries for efficient path planning.
     */
    class Field {
      public:
        // Forward declaration of test functions
        friend datapod::Geo get_field_datum(const Field &field);
        friend double get_total_field_area(const Field &field);

      private:
        datapod::Polygon border_;
        std::vector<Part> parts_;
        datapod::Geo datum_{};
        std::mt19937 rnd_;
        double overlap_threshold_{0.7};

      public:
        /**
         * @brief Construct a Field from a border polygon
         *
         * @param border The field boundary polygon
         * @param datum Geographic reference datum
         * @param centred Whether to center the field (unused, kept for API compatibility)
         * @param area_threshold Area threshold for subdivision (unused without Partitioner)
         * @param use_equal_areas Whether to use equal area partitioning (unused without Partitioner)
         */
        inline Field(const datapod::Polygon &border, const datapod::Geo &datum, bool centred = true,
                     double area_threshold = 0.5, bool use_equal_areas = false)
            : border_(border), datum_(datum) {
            // Without concord::Partitioner, we treat the entire field as a single part
            // Future: implement custom partitioning if needed

            (void)centred;         // Suppress unused parameter warning
            (void)area_threshold;  // Suppress unused parameter warning
            (void)use_equal_areas; // Suppress unused parameter warning

            // Create a single part from the entire border
            Part p;
            p.boundary = create_ring(border_);
            parts_.push_back(std::move(p));
        }

        const std::vector<Part> &get_parts() const { return parts_; }
        std::vector<Part> &get_parts() { return parts_; }
        const datapod::Polygon &get_border() const { return border_; }

        /**
         * @brief Generate field geometry (headlands and swaths)
         *
         * @param swath_width Width of each swath
         * @param angle_degrees Swath angle in degrees (0 = auto-optimize)
         * @param headland_count Number of headland passes
         */
        inline void gen_field(double swath_width, double angle_degrees = 0, int headland_count = 1) {
            for (auto &part : parts_) {
                part.headlands.clear();
                part.swaths.clear();
                part.headlands = generate_headlands(part.boundary.polygon, swath_width, headland_count);

                // Safe access to headlands - use boundary if no headlands were generated
                datapod::Polygon interior = (headland_count > 0 && !part.headlands.empty())
                                                ? part.headlands.back().polygon
                                                : part.boundary.polygon;

                part.swaths = generate_swaths(swath_width, angle_degrees, interior);

                // Rebuild R-trees after generating new geometry
                part.rebuild_rtrees();
            }
        }

        /**
         * @brief Find swaths near a query point
         *
         * @param part_idx Index of the part to search
         * @param query_point Query point
         * @param radius Search radius
         * @return Vector of swath indices within radius
         */
        inline std::vector<std::size_t> find_nearby_swaths(std::size_t part_idx, const datapod::Point &query_point,
                                                           double radius) const {
            if (part_idx >= parts_.size())
                return {};

            const auto &part = parts_[part_idx];
            std::vector<std::size_t> result;

            // Create search box around query point
            datapod::AABB search_box{
                datapod::Point{query_point.x - radius, query_point.y - radius, query_point.z - radius},
                datapod::Point{query_point.x + radius, query_point.y + radius, query_point.z + radius}};

            // Query R-tree
            auto candidates = part.swath_rtree.query_intersects(search_box);

            // Filter by actual distance
            for (const auto &candidate : candidates) {
                std::size_t idx = candidate.data;
                if (idx < part.swaths.size()) {
                    double dist = part.swaths[idx].line.distance_to(query_point);
                    if (dist <= radius) {
                        result.push_back(idx);
                    }
                }
            }

            return result;
        }

        /**
         * @brief Find nearest swath endpoints to a query point
         *
         * @param part_idx Index of the part to search
         * @param query_point Query point
         * @param k Number of nearest neighbors to find
         * @return Vector of endpoint indices (even = start, odd = end)
         */
        inline std::vector<std::size_t>
        find_nearest_swath_endpoints(std::size_t part_idx, const datapod::Point &query_point, std::size_t k = 1) const {
            if (part_idx >= parts_.size())
                return {};

            const auto &part = parts_[part_idx];
            auto nearest = part.swath_endpoints_rtree.query_nearest(query_point, k);

            std::vector<std::size_t> result;
            for (const auto &entry : nearest) {
                result.push_back(entry.data);
            }
            return result;
        }

        /**
         * @brief Find optimal swath traversal order using nearest neighbor heuristic
         *
         * @param part_idx Index of the part
         * @param start_point Starting position
         * @return Vector of swath indices in optimal order
         */
        inline std::vector<std::size_t> optimize_swath_order(std::size_t part_idx,
                                                             const datapod::Point &start_point) const {
            if (part_idx >= parts_.size())
                return {};

            const auto &part = parts_[part_idx];
            if (part.swaths.empty())
                return {};

            std::vector<std::size_t> order;
            std::vector<bool> visited(part.swaths.size(), false);
            datapod::Point current_pos = start_point;

            for (std::size_t i = 0; i < part.swaths.size(); ++i) {
                double min_dist = std::numeric_limits<double>::max();
                std::size_t next_swath = 0;

                // Find closest unvisited swath
                for (std::size_t j = 0; j < part.swaths.size(); ++j) {
                    if (visited[j])
                        continue;

                    double dist_to_start = current_pos.distance_to(part.swaths[j].line.start);
                    double dist_to_end = current_pos.distance_to(part.swaths[j].line.end);

                    double min_swath_dist = std::min(dist_to_start, dist_to_end);
                    if (min_swath_dist < min_dist) {
                        min_dist = min_swath_dist;
                        next_swath = j;
                    }
                }

                visited[next_swath] = true;
                order.push_back(next_swath);

                // Update current position to end of chosen swath
                current_pos = part.swaths[next_swath].line.end;
            }

            return order;
        }

      private:
        /**
         * @brief Generate headland rings by shrinking the polygon
         *
         * @param polygon The polygon to generate headlands from
         * @param shrink_dist Distance to shrink for each headland
         * @param count Number of headland rings to generate
         * @return Vector of headland rings
         */
        inline std::vector<Ring> generate_headlands(const datapod::Polygon &polygon, double shrink_dist,
                                                    int count) const {
            // Quick input validation
            if (count <= 0)
                return {}; // No headlands requested

            if (shrink_dist < 0)
                throw std::invalid_argument("negative shrink");

            // Ensure shrink distance is positive
            double actual_shrink_dist = std::max(0.1, shrink_dist);

            // Initialize result vector
            std::vector<Ring> H;

            // Ensure the polygon has points
            if (polygon.vertices.size() < 3) {
                std::cerr << "Warning: Invalid polygon for headland generation" << std::endl;
                return H;
            }

            // Correct the polygon winding order
            datapod::Polygon base = temp::correct_polygon(polygon);

            // Generate each headland as a shrunken version of the previous one
            for (int i = 0; i < count; ++i) {
                // Get the current polygon to shrink (either the base field or the last headland)
                datapod::Polygon current = (i == 0) ? base : H.back().polygon;

                // Shrink the polygon using temp::shrink_polygon
                datapod::Polygon shrunk = temp::shrink_polygon(current, actual_shrink_dist);

                // Check if buffer operation produced valid results
                if (shrunk.vertices.size() < 3) {
                    std::cerr << "Warning: empty buffer result at headland " << i << std::endl;
                    break;
                }

                // Check if area is too small
                double shrunk_area = shrunk.area();
                if (shrunk_area < 1e-6) {
                    std::cerr << "Warning: headland " << i << " has negligible area" << std::endl;
                    break;
                }

                // Clean up the polygon by removing colinear points
                datapod::Polygon simp = utils::remove_colinear_points(shrunk, 1e-4);

                // Ensure polygon is closed
                if (!simp.vertices.empty()) {
                    const auto &first = simp.vertices.front();
                    const auto &last = simp.vertices.back();
                    double dx = first.x - last.x;
                    double dy = first.y - last.y;
                    if (dx * dx + dy * dy > 1e-10) {
                        simp.vertices.push_back(first);
                    }
                }

                // Create a ring from the polygon and add to results
                H.push_back(create_ring(std::move(simp)));
            }

            return H;
        }

        /**
         * @brief Generate swaths within a polygon boundary
         *
         * @param swath_width Width of each swath
         * @param angle_deg Swath angle in degrees (0 = auto-optimize)
         * @param border The polygon boundary to fill with swaths
         * @return Vector of swaths
         */
        inline std::vector<Swath> generate_swaths(double swath_width, double angle_deg,
                                                  const datapod::Polygon &border) const {
            // Auto-optimize angle if 0
            if (angle_deg == 0.0) {
                std::vector<Swath> best_out;
                std::size_t best_count = std::numeric_limits<std::size_t>::max();
                for (int deg = 1; deg < 360; ++deg) {
                    auto out = generate_swaths(swath_width, static_cast<double>(deg), border);
                    if (out.size() < best_count) {
                        best_count = out.size();
                        best_out = std::move(out);
                    }
                }
                return best_out;
            }

            std::vector<Swath> out;

            // Validate polygon
            if (!temp::is_valid_polygon(border)) {
                datapod::Polygon corrected = temp::correct_polygon(border);
                if (!temp::is_valid_polygon(corrected)) {
                    std::cout << "Error: Unable to create valid polygon for swath generation" << std::endl;
                    return out;
                }
                // Use corrected polygon
                return generate_swaths(swath_width, angle_deg, corrected);
            }

            double rad = angle_deg * M_PI / 180.0;

            // Calculate centroid
            datapod::Point centroid_pt = temp::centroid(border);
            double cx = centroid_pt.x;
            double cy = centroid_pt.y;

            double cosA = std::cos(rad);
            double sinA = std::sin(rad);

            // Get bounding box
            datapod::AABB bbox = border.get_aabb();
            double width = bbox.max_point.x - bbox.min_point.x;
            double height = bbox.max_point.y - bbox.min_point.y;

            // Use more reasonable bounds for line generation
            double line_ext = std::max(width, height) * 1.5;    // Extension for lines
            double max_offset = std::max(width, height) * 0.75; // Maximum offset from center

            for (double offs = -max_offset; offs <= max_offset; offs += swath_width) {
                double x1 = cx + offs * sinA - line_ext * cosA;
                double y1 = cy - offs * cosA - line_ext * sinA;
                double x2 = cx + offs * sinA + line_ext * cosA;
                double y2 = cy - offs * cosA + line_ext * sinA;

                // Create a segment for clipping
                datapod::Segment ray{datapod::Point{x1, y1, 0.0}, datapod::Point{x2, y2, 0.0}};

                // Clip segment to polygon
                std::vector<datapod::Segment> clips = temp::clip_segment_to_polygon(ray, border);

                for (const auto &seg : clips) {
                    double seg_length = seg.length();
                    if (seg_length < swath_width * 0.1)
                        continue;

                    out.push_back(create_swath(seg.start, seg.end, SwathType::Swath));
                }
            }

            return out;
        }
    };

} // namespace farmtrax
