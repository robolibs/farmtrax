#pragma once

#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cmath>
#include <concord/concord.hpp>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace farmtrax {
    // Type definitions (same as in field.hpp)
    using BPoint = boost::geometry::model::d2::point_xy<double>;
    using BLineString = boost::geometry::model::linestring<BPoint>;
    using BPolygon = boost::geometry::model::polygon<BPoint>;

    namespace utils {
        uint8_t float_to_byte(float v, float min = 0.0f, float max = 255.0f);

        // Calculate distance from a point to a line segment
        double point_to_line_distance(const BPoint &point, const BPoint &line_start, const BPoint &line_end);

        // Calculate angle between two vectors represented as BPoints (relative to origin)
        double angle_between(const BPoint &v1, const BPoint &v2);

        bool are_colinear(const concord::Point &p1, const concord::Point &p2, const concord::Point &p3,
                          double epsilon = 1e-10);

        concord::Polygon remove_colinear_points(const concord::Polygon &polygon, double epsilon = 0.01);

        // Conversion functions between concord and boost geometry types
        BPoint to_boost(const concord::Point &in);

        concord::Point from_boost(const BPoint &in, const concord::Datum &datum = concord::Datum{});

        BLineString to_boost(const concord::Line &L);

        concord::Line from_boost(const BLineString &L, const concord::Datum &datum = concord::Datum{});

        BPolygon to_boost(const concord::Polygon &poly);

        concord::Polygon from_boost(const BPolygon &poly, const concord::Datum &datum = concord::Datum{});
    } // namespace utils

} // namespace farmtrax
