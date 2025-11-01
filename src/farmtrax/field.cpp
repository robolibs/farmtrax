#include "farmtrax/field.hpp"

namespace farmtrax {

    Ring create_ring(const concord::Polygon &poly, std::string uuid) {
        if (uuid.empty())
            uuid = boost::uuids::to_string(boost::uuids::random_generator()());
        Ring ring = {poly, uuid};
        // Convert to boost polygon and compute bounding box
        ring.b_polygon = boost::geometry::model::polygon<BPoint>();
        for (auto const &pt : poly.getPoints()) {
            ring.b_polygon.outer().emplace_back(pt.x, pt.y);
        }
        if (!boost::geometry::equals(ring.b_polygon.outer().front(), ring.b_polygon.outer().back()))
            ring.b_polygon.outer().push_back(ring.b_polygon.outer().front());
        boost::geometry::correct(ring.b_polygon);
        ring.bounding_box = boost::geometry::return_envelope<BBox>(ring.b_polygon);
        return ring;
    }

    Swath create_swath(const concord::Point &start, const concord::Point &end, SwathType type, std::string uuid) {
        concord::Line L;
        L.setStart(start);
        L.setEnd(end);
        if (uuid.empty())
            uuid = boost::uuids::to_string(boost::uuids::random_generator()());

        Swath swath = {L, uuid, type};
        // Convert to boost linestring and compute bounding box
        swath.b_line.emplace_back(start.x, start.y);
        swath.b_line.emplace_back(end.x, end.y);
        swath.bounding_box = boost::geometry::return_envelope<BBox>(swath.b_line);
        return swath;
    }

    Field::Field(const concord::Polygon &border, const concord::Datum &datum, bool centred, double area_threshold,
                 bool use_equal_areas)
        : border_(border), datum_(datum) {
        partitioner_ = concord::Partitioner(border_);

        std::vector<concord::Polygon> divisions;

        if (use_equal_areas) {
            // Use the new equal-area partitioning method
            divisions = partitioner_.partition_equal_areas(area_threshold, 0.25); // 25% tolerance
        } else {
            // Use the original max-area partitioning
            concord::Partitioner::PartitionCriteria criteria;
            criteria.max_area = area_threshold;
            criteria.max_aspect_ratio = 2.5;
            criteria.min_convexity = 0.7;
            criteria.tooth_threshold = 0.2;
            criteria.enable_bridge_detection = true;
            criteria.enable_tooth_detection = true;
            criteria.enable_aspect_splitting = true;

            divisions = partitioner_.partition(area_threshold, criteria);
            std::cout << "Split " << divisions.size() << " parts\n";
        }

        parts_.reserve(divisions.size());
        for (auto const &poly : divisions) {
            Part p;
            p.boundary = create_ring(poly);
            parts_.push_back(std::move(p));
        }
    }

    void Field::gen_field(double swath_width, double angle_degrees, int headland_count) {
        for (auto &part : parts_) {
            part.headlands.clear();
            part.swaths.clear();
            part.headlands = generate_headlands(part.boundary.polygon, swath_width, headland_count);

            // Safe access to headlands - use boundary if no headlands were generated
            concord::Polygon interior =
                (headland_count > 0 && !part.headlands.empty()) ? part.headlands.back().polygon : part.boundary.polygon;

            part.swaths = generate_swaths(swath_width, angle_degrees, interior);

            // Rebuild R-trees after generating new geometry
            part.rebuild_rtrees();
        }
    }

    std::vector<std::size_t> Field::find_nearby_swaths(std::size_t part_idx, const BPoint &query_point,
                                                       double radius) const {
        if (part_idx >= parts_.size())
            return {};

        const auto &part = parts_[part_idx];
        std::vector<std::size_t> result;

        // Create search box around query point
        BBox search_box(BPoint(query_point.x() - radius, query_point.y() - radius),
                        BPoint(query_point.x() + radius, query_point.y() + radius));

        // Query R-tree
        std::vector<SwathRTreeValue> candidates;
        part.swath_rtree.query(boost::geometry::index::intersects(search_box), std::back_inserter(candidates));

        // Filter by actual distance
        for (const auto &candidate : candidates) {
            std::size_t idx = candidate.second;
            if (idx < part.swaths.size()) {
                double dist = boost::geometry::distance(query_point, part.swaths[idx].b_line);
                if (dist <= radius) {
                    result.push_back(idx);
                }
            }
        }

        return result;
    }

    std::vector<std::size_t> Field::find_nearest_swath_endpoints(std::size_t part_idx, const BPoint &query_point,
                                                                 std::size_t k) const {
        if (part_idx >= parts_.size())
            return {};

        const auto &part = parts_[part_idx];
        std::vector<PointRTreeValue> nearest;
        part.swath_endpoints_rtree.query(boost::geometry::index::nearest(query_point, k), std::back_inserter(nearest));

        std::vector<std::size_t> result;
        for (const auto &point_value : nearest) {
            result.push_back(point_value.second);
        }
        return result;
    }

    std::vector<std::size_t> Field::optimize_swath_order(std::size_t part_idx, const BPoint &start_point) const {
        if (part_idx >= parts_.size())
            return {};

        const auto &part = parts_[part_idx];
        if (part.swaths.empty())
            return {};

        std::vector<std::size_t> order;
        std::vector<bool> visited(part.swaths.size(), false);
        BPoint current_pos = start_point;

        for (std::size_t i = 0; i < part.swaths.size(); ++i) {
            double min_dist = std::numeric_limits<double>::max();
            std::size_t next_swath = 0;

            // Use R-tree to find nearby candidates first
            auto nearby = find_nearby_swaths(part_idx, current_pos, min_dist);

            // Find closest unvisited swath
            for (std::size_t j = 0; j < part.swaths.size(); ++j) {
                if (visited[j])
                    continue;

                double dist_to_start = boost::geometry::distance(
                    current_pos, BPoint(part.swaths[j].line.getStart().x, part.swaths[j].line.getStart().y));
                double dist_to_end = boost::geometry::distance(
                    current_pos, BPoint(part.swaths[j].line.getEnd().x, part.swaths[j].line.getEnd().y));

                double min_swath_dist = std::min(dist_to_start, dist_to_end);
                if (min_swath_dist < min_dist) {
                    min_dist = min_swath_dist;
                    next_swath = j;
                }
            }

            visited[next_swath] = true;
            order.push_back(next_swath);

            // Update current position to end of chosen swath
            current_pos = BPoint(part.swaths[next_swath].line.getEnd().x, part.swaths[next_swath].line.getEnd().y);
        }

        return order;
    }

    std::vector<Ring> Field::generate_headlands(const concord::Polygon &polygon, double shrink_dist, int count) const {
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
        if (polygon.getPoints().size() < 3) {
            std::cerr << "Warning: Invalid polygon for headland generation" << std::endl;
            return H;
        }

        // Convert to boost polygon with careful error handling
        BPolygon base;
        try {
            base = utils::to_boost(polygon);
            if (base.outer().size() < 3) {
                std::cerr << "Warning: Invalid boost polygon for headland generation" << std::endl;
                return H;
            }
        } catch (const std::exception &e) {
            std::cerr << "Boost conversion error: " << e.what() << std::endl;
            return H;
        }

        // Generate each headland as a shrunken version of the previous one
        for (int i = 0; i < count; ++i) {
            // Get the current polygon to shrink (either the base field or the last headland)
            BPolygon current;
            try {
                current = (i == 0 ? base : utils::to_boost(H.back().polygon));
            } catch (const std::exception &e) {
                std::cerr << "Error getting polygon for headland " << i << ": " << e.what() << std::endl;
                break;
            }

            // Set up buffer operation parameters
            boost::geometry::model::multi_polygon<BPolygon> buf;
            boost::geometry::strategy::buffer::distance_symmetric<double> dist(-actual_shrink_dist);
            boost::geometry::strategy::buffer::side_straight side;
            boost::geometry::strategy::buffer::join_miter join;
            boost::geometry::strategy::buffer::end_flat end;
            boost::geometry::strategy::buffer::point_square point;

            // Attempt to buffer (shrink) the polygon
            try {
                boost::geometry::buffer(current, buf, dist, side, join, end, point);
            } catch (const std::exception &e) {
                std::cerr << "Buffer operation failed for headland " << i << ": " << e.what() << std::endl;
                break;
            }

            // Check if buffer operation produced any results
            if (buf.empty()) {
                std::cerr << "Warning: empty buffer result at headland " << i << std::endl;
                // Instead of breaking, we can return what we have so far
                break;
            }

            // Find the polygon with the largest area (likely the main interior)
            const BPolygon *best = nullptr;
            double maxA = -std::numeric_limits<double>::max();

            for (const auto &cand : buf) {
                try {
                    double a = std::abs(boost::geometry::area(cand)); // Use absolute area
                    if (a > maxA && a > 0.0001) {                     // Ensure minimum area
                        maxA = a;
                        best = &cand;
                    }
                } catch (const std::exception &e) {
                    std::cerr << "Area calculation error: " << e.what() << std::endl;
                    continue;
                }
            }

            // Check if we found a valid polygon
            if (!best || maxA <= 0.0001) {
                std::cerr << "Warning: no valid polygon found at headland " << i << std::endl;
                break;
            }

            try {
                // Convert back to concord polygon
                concord::Polygon tmp = utils::from_boost(*best, datum_);

                // Clean up the polygon by removing colinear points
                concord::Polygon simp = utils::remove_colinear_points(tmp, 1e-4);

                // Ensure polygon is closed
                if (!simp.getPoints().empty() && (simp.getPoints().front().x != simp.getPoints().back().x ||
                                                  simp.getPoints().front().y != simp.getPoints().back().y)) {
                    simp.addPoint(simp.getPoints().front());
                }

                // Create a ring from the polygon and add to results
                H.push_back(create_ring(std::move(simp)));
            } catch (const std::exception &e) {
                std::cerr << "Error creating headland " << i << ": " << e.what() << std::endl;
                break;
            }
        }
        return H;
    }

    std::vector<Swath> Field::generate_swaths(double swath_width, double angle_deg,
                                              const concord::Polygon &border) const {
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
        BPolygon bounds = utils::to_boost(border);

        // Ensure polygon is valid and correctly oriented
        if (!boost::geometry::is_valid(bounds)) {
            std::cout << "Warning: Invalid polygon, attempting to correct" << std::endl;
            boost::geometry::correct(bounds);
            if (!boost::geometry::is_valid(bounds)) {
                std::cout << "Error: Unable to create valid polygon for swath generation" << std::endl;
                return out;
            }
        }

        // Debug: Check polygon area and orientation
        double area = boost::geometry::area(bounds);
        // std::cout << "Polygon area: " << area << " (should be positive for correct orientation)" << std::endl;

        double rad = angle_deg * M_PI / 180.0;
        BPoint centroid;
        boost::geometry::centroid(bounds, centroid);
        double cx = centroid.x();
        double cy = centroid.y();
        double cosA = std::cos(rad);
        double sinA = std::sin(rad);
        auto bbox = boost::geometry::return_envelope<boost::geometry::model::box<BPoint>>(bounds);
        double width = bbox.max_corner().x() - bbox.min_corner().x();
        double height = bbox.max_corner().y() - bbox.min_corner().y();

        // Use more reasonable bounds for line generation
        double line_ext = std::max(width, height) * 1.5;    // Extension for lines
        double max_offset = std::max(width, height) * 0.75; // Maximum offset from center

        // std::cout << "Generating swaths: centroid=(" << cx << "," << cy << "), angle=" << angle_deg
        //           << "°, line_ext=" << line_ext << ", max_offset=" << max_offset << std::endl;

        int swath_count = 0;
        for (double offs = -max_offset; offs <= max_offset; offs += swath_width) {
            double x1 = cx + offs * sinA - line_ext * cosA;
            double y1 = cy - offs * cosA - line_ext * sinA;
            double x2 = cx + offs * sinA + line_ext * cosA;
            double y2 = cy - offs * cosA + line_ext * sinA;
            BLineString ray;
            ray.emplace_back(x1, y1);
            ray.emplace_back(x2, y2);
            std::vector<BLineString> clips;
            boost::geometry::intersection(ray, bounds, clips);

            for (auto const &seg : clips) {
                if (boost::geometry::length(seg) < swath_width * 0.1)
                    continue;
                swath_count++;
                concord::Point a{seg.front().x(), seg.front().y(), 0};
                concord::Point b{seg.back().x(), seg.back().y(), 0};
                out.push_back(create_swath(a, b, SwathType::Swath));
            }
        }

        // std::cout << "Generated " << swath_count << " swaths inside polygon" << std::endl;

        return out;
    }

} // namespace farmtrax