#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <concord/concord.hpp>
#include <concord/geometry/polygon/partitioner.hpp>

#include "farmtrax/utils/utils.hpp"

namespace farmtrax {
    using BPoint = boost::geometry::model::d2::point_xy<double>;
    using BLineString = boost::geometry::model::linestring<BPoint>;
    using BPolygon = boost::geometry::model::polygon<BPoint>;
    using BBox = boost::geometry::model::box<BPoint>;

    // R-tree value types for different geometry types
    using SwathRTreeValue = std::pair<BBox, std::size_t>;   // Bounding box + swath index
    using RingRTreeValue = std::pair<BBox, std::size_t>;    // Bounding box + ring index
    using PointRTreeValue = std::pair<BPoint, std::size_t>; // Point + index

    // R-tree types
    using SwathRTree = boost::geometry::index::rtree<SwathRTreeValue, boost::geometry::index::quadratic<16>>;
    using RingRTree = boost::geometry::index::rtree<RingRTreeValue, boost::geometry::index::quadratic<16>>;
    using PointRTree = boost::geometry::index::rtree<PointRTreeValue, boost::geometry::index::quadratic<16>>;

    struct Ring {
        concord::Polygon polygon;
        std::string uuid;
        bool finished = false;
        BPolygon b_polygon;
        BBox bounding_box; // Add bounding box for R-tree
    };

    Ring create_ring(const concord::Polygon &poly, std::string uuid = "");

    enum class SwathType {
        Swath,
        Connection,
        Around,
        Headland,
    };

    struct Swath {
        concord::Line line;
        std::string uuid;
        SwathType type = SwathType::Swath; // Default type is Swath
        bool finished = false;
        BLineString b_line;
        BBox bounding_box; // Add bounding box for R-tree

        // Additional fields needed for testing
        int id = -1;
        double width = 0.0;
        std::vector<concord::Point> points;
        std::vector<BPoint> centerline;

        // Get the head (start) point
        concord::Point getHead() const { return line.getStart(); }

        // Get the tail (end) point
        concord::Point getTail() const { return line.getEnd(); }

        // Swap head and tail (reverse direction)
        void swapDirection() {
            concord::Point temp = line.getStart();
            line.setStart(line.getEnd());
            line.setEnd(temp);
            // Update boost linestring and bounding box
            b_line.clear();
            b_line.emplace_back(line.getStart().x, line.getStart().y);
            b_line.emplace_back(line.getEnd().x, line.getEnd().y);
            bounding_box = boost::geometry::return_envelope<BBox>(b_line);
        }

        // Create a copy with swapped direction
        Swath withSwappedDirection() const {
            Swath swapped = *this;
            swapped.swapDirection();
            return swapped;
        }
    };

    Swath create_swath(const concord::Point &start, const concord::Point &end, SwathType type, std::string uuid = "");

    struct Part {
        Ring boundary; // The boundary polygon of this subdivided part
        std::vector<Swath> swaths;
        std::vector<Ring> headlands;

        // R-trees for spatial indexing
        SwathRTree swath_rtree;
        RingRTree headland_rtree;
        PointRTree swath_endpoints_rtree; // For start/end points of swaths

        void rebuild_rtrees() {
            // Clear existing trees
            swath_rtree.clear();
            headland_rtree.clear();
            swath_endpoints_rtree.clear();

            // Rebuild swath R-tree
            for (std::size_t i = 0; i < swaths.size(); ++i) {
                swath_rtree.insert(std::make_pair(swaths[i].bounding_box, i));

                // Add start and end points to point R-tree
                BPoint start_pt(swaths[i].line.getStart().x, swaths[i].line.getStart().y);
                BPoint end_pt(swaths[i].line.getEnd().x, swaths[i].line.getEnd().y);
                swath_endpoints_rtree.insert(std::make_pair(start_pt, i * 2));   // Even indices for start points
                swath_endpoints_rtree.insert(std::make_pair(end_pt, i * 2 + 1)); // Odd indices for end points
            }

            // Rebuild headland R-tree
            for (std::size_t i = 0; i < headlands.size(); ++i) {
                headland_rtree.insert(std::make_pair(headlands[i].bounding_box, i));
            }
        }
    };

    class Field {
      public:
        // Forward declaration of test functions
        friend concord::Datum get_field_datum(const Field &field);
        friend double get_total_field_area(const Field &field);

      private:
        concord::Polygon border_;
        std::vector<Part> parts_;

        concord::Partitioner partitioner_;
        concord::Datum datum_{};
        std::mt19937 rnd_;
        double overlap_threshold_{0.7};

      public:
        Field(const concord::Polygon &border, const concord::Datum &datum, bool centred = true,
              double area_threshold = 0.5, bool use_equal_areas = false);

        const std::vector<Part> &get_parts() const { return parts_; }
        std::vector<Part> &get_parts() { return parts_; }
        const concord::Polygon &get_border() const { return border_; }

        void gen_field(double swath_width, double angle_degrees = 0, int headland_count = 1);

        // Spatial query methods using R-tree
        std::vector<std::size_t> find_nearby_swaths(std::size_t part_idx, const BPoint &query_point,
                                                    double radius) const;

        std::vector<std::size_t> find_nearest_swath_endpoints(std::size_t part_idx, const BPoint &query_point,
                                                              std::size_t k = 1) const;

        // Find optimal swath traversal order using R-tree for nearest neighbor queries
        std::vector<std::size_t> optimize_swath_order(std::size_t part_idx, const BPoint &start_point) const;

      private:
        std::vector<Ring> generate_headlands(const concord::Polygon &polygon, double shrink_dist, int count) const;

        std::vector<Swath> generate_swaths(double swath_width, double angle_deg, const concord::Polygon &border) const;
    };

} // namespace farmtrax
