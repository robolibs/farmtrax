#include "farmtrax/utils/utils.hpp"

namespace farmtrax {

    namespace utils {
        uint8_t float_to_byte(float v, float min, float max) {
            v = std::clamp(v, 0.0f, 1.0f);
            float scaled = v * 255.0f;
            float clamped = std::clamp(scaled, min, max);
            return static_cast<uint8_t>(std::round(clamped));
        }

        // Calculate distance from a point to a line segment
        double point_to_line_distance(const BPoint &point, const BPoint &line_start, const BPoint &line_end) {
            // If line is a point, return distance to that point
            if (boost::geometry::equals(line_start, line_end)) {
                return boost::geometry::distance(point, line_start);
            }

            // Vector from line_start to line_end
            double line_vec_x = boost::geometry::get<0>(line_end) - boost::geometry::get<0>(line_start);
            double line_vec_y = boost::geometry::get<1>(line_end) - boost::geometry::get<1>(line_start);

            // Vector from line_start to point
            double point_vec_x = boost::geometry::get<0>(point) - boost::geometry::get<0>(line_start);
            double point_vec_y = boost::geometry::get<1>(point) - boost::geometry::get<1>(line_start);

            // Length of line segment squared
            double line_len_squared = line_vec_x * line_vec_x + line_vec_y * line_vec_y;

            // Calculate projection of point onto line
            double t = (point_vec_x * line_vec_x + point_vec_y * line_vec_y) / line_len_squared;

            // If projection is outside the line segment, return distance to the nearest endpoint
            if (t < 0.0)
                return boost::geometry::distance(point, line_start);
            if (t > 1.0)
                return boost::geometry::distance(point, line_end);

            // Calculate the projected point on the line
            BPoint projected_point(boost::geometry::get<0>(line_start) + t * line_vec_x,
                                   boost::geometry::get<1>(line_start) + t * line_vec_y);

            // Return distance from point to projected point
            return boost::geometry::distance(point, projected_point);
        }

        // Calculate angle between two vectors represented as BPoints (relative to origin)
        double angle_between(const BPoint &v1, const BPoint &v2) {
            // Extract vector components
            double v1_x = boost::geometry::get<0>(v1);
            double v1_y = boost::geometry::get<1>(v1);
            double v2_x = boost::geometry::get<0>(v2);
            double v2_y = boost::geometry::get<1>(v2);

            // Calculate magnitudes
            double v1_mag = std::sqrt(v1_x * v1_x + v1_y * v1_y);
            double v2_mag = std::sqrt(v2_x * v2_x + v2_y * v2_y);

            // Prevent division by zero
            if (v1_mag < 1e-10 || v2_mag < 1e-10) {
                return 0.0; // One of the vectors is zero-length
            }

            // Calculate dot product and normalize
            double dot_product = (v1_x * v2_x + v1_y * v2_y) / (v1_mag * v2_mag);

            // Ensure dot product is within valid range for acos
            dot_product = std::clamp(dot_product, -1.0, 1.0);

            // Return angle in radians
            return std::acos(dot_product);
        }

        bool are_colinear(const concord::Point &p1, const concord::Point &p2, const concord::Point &p3,
                          double epsilon) {
            using BPoint = boost::geometry::model::d2::point_xy<double>;
            BPoint a{p1.x, p1.y}, b{p2.x, p2.y}, c{p3.x, p3.y};
            double area = (boost::geometry::get<0>(a) * (boost::geometry::get<1>(b) - boost::geometry::get<1>(c)) +
                           boost::geometry::get<0>(b) * (boost::geometry::get<1>(c) - boost::geometry::get<1>(a)) +
                           boost::geometry::get<0>(c) * (boost::geometry::get<1>(a) - boost::geometry::get<1>(b))) *
                          0.5;
            return std::abs(area) < epsilon;
        }

        concord::Polygon remove_colinear_points(const concord::Polygon &polygon, double epsilon) {
            concord::Polygon result;
            auto const &pts = polygon.getPoints();
            if (pts.size() < 4)
                return polygon;
            size_t n = pts.size() - 1;
            for (size_t i = 0; i < n; ++i) {
                auto const &prev = pts[(i + n - 1) % n];
                auto const &curr = pts[i];
                auto const &next = pts[(i + 1) % n];
                if (!are_colinear(prev, curr, next, epsilon))
                    result.addPoint(curr);
            }
            result.addPoint(pts.front());
            return result;
        }

        // Conversion functions between concord and boost geometry types
        BPoint to_boost(const concord::Point &in) { return BPoint{in.x, in.y}; }

        concord::Point from_boost(const BPoint &in, const concord::Datum &datum) {
            return concord::Point{boost::geometry::get<0>(in), boost::geometry::get<1>(in), 0.0};
        }

        BLineString to_boost(const concord::Line &L) {
            BLineString out;
            out.emplace_back(to_boost(L.getStart()));
            out.emplace_back(to_boost(L.getEnd()));
            return out;
        }

        concord::Line from_boost(const BLineString &L, const concord::Datum &datum) {
            concord::Line out;
            out.setStart(from_boost(L.front(), datum));
            out.setEnd(from_boost(L.back(), datum));
            return out;
        }

        BPolygon to_boost(const concord::Polygon &poly) {
            BPolygon out;
            const auto &points = poly.getPoints();

            // Skip the last point if it's the same as the first (it's a closing point)
            size_t n = points.size();
            size_t limit = (n > 0 && std::abs(points.front().x - points.back().x) < 1e-10 &&
                            std::abs(points.front().y - points.back().y) < 1e-10)
                               ? n - 1
                               : n;

            for (size_t i = 0; i < limit; ++i) {
                out.outer().emplace_back(to_boost(points[i]));
            }

            // Make sure polygon is closed
            if (!out.outer().empty()) {
                if (!boost::geometry::equals(out.outer().front(), out.outer().back())) {
                    out.outer().push_back(out.outer().front());
                }
            }

            // Correct the polygon to ensure proper orientation and validity
            // This is essential for intersection operations to work correctly
            boost::geometry::correct(out);

            return out;
        }

        concord::Polygon from_boost(const BPolygon &poly, const concord::Datum &datum) {
            concord::Polygon out;

            // Skip the last point if it's the same as the first (Boost polygons are typically closed)
            size_t n = poly.outer().size();
            if (n == 0)
                return out; // Empty polygon

            // Compare using coordinate values directly instead of boost::geometry::equals
            size_t limit = (n > 1 &&
                            std::abs(boost::geometry::get<0>(poly.outer().front()) -
                                     boost::geometry::get<0>(poly.outer().back())) < 1e-10 &&
                            std::abs(boost::geometry::get<1>(poly.outer().front()) -
                                     boost::geometry::get<1>(poly.outer().back())) < 1e-10)
                               ? n - 1
                               : n;

            // Use the original polygon to preserve point order
            // Don't use boost::geometry::correct() as it can change point order
            for (size_t i = 0; i < limit; ++i) {
                out.addPoint(from_boost(poly.outer()[i], datum));
            }

            // Make sure the polygon is closed by adding the first point at the end
            if (!out.getPoints().empty() && (std::abs(out.getPoints().front().x - out.getPoints().back().x) > 1e-10 ||
                                             std::abs(out.getPoints().front().y - out.getPoints().back().y) > 1e-10)) {
                out.addPoint(out.getPoints().front());
            }

            return out;
        }
    } // namespace utils

} // namespace farmtrax
