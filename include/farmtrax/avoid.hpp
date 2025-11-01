#pragma once

#include "farmtrax/field.hpp"
#include "farmtrax/utils/utils.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/touches.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <memory>
#include <vector>

namespace farmtrax {

    class ObstacleAvoider {
        std::vector<BPolygon> obstacles_;
        std::vector<BPolygon> inflated_obstacles_;
        float inflation_distance_;
        concord::Datum datum_;

      public:
        // Constructor takes array of polygons as obstacles
        explicit ObstacleAvoider(const std::vector<BPolygon> &obstacles,
                                 const concord::Datum &datum = concord::Datum{});

        // Constructor with concord::Polygon obstacles
        explicit ObstacleAvoider(const std::vector<concord::Polygon> &obstacles,
                                 const concord::Datum &datum = concord::Datum{});

        // Main avoidance method
        std::vector<std::shared_ptr<Swath>> avoid(const std::vector<std::shared_ptr<const Swath>> &input_swaths,
                                                  float inflation_distance);

        // Overload for non-const swaths
        std::vector<std::shared_ptr<Swath>> avoid(const std::vector<std::shared_ptr<Swath>> &input_swaths,
                                                  float inflation_distance);

        // Get original obstacles
        const std::vector<BPolygon> &get_obstacles() const;

        // Get inflated obstacles
        const std::vector<BPolygon> &get_inflated_obstacles() const;

      private:
        // Inflate all obstacles by the given distance
        void inflate_obstacles();

        // Inflate a single polygon
        BPolygon inflate_polygon(const BPolygon &polygon, float distance) const;

        // Process a single swath against all obstacles
        std::vector<std::shared_ptr<Swath>> process_swath(const std::shared_ptr<const Swath> &swath);

        // Cut swath around obstacles
        std::vector<std::shared_ptr<Swath>> cut_swath_around_obstacles(const std::shared_ptr<const Swath> &swath);
    };

} // namespace farmtrax