#include "doctest/doctest.h"

#include "farmtrax/farmtrax.hpp"

namespace {
    datapod::Polygon create_test_polygon() {
        datapod::Polygon poly;
        poly.vertices.push_back(datapod::Point{0.0, 0.0, 0.0});
        poly.vertices.push_back(datapod::Point{100.0, 0.0, 0.0});
        poly.vertices.push_back(datapod::Point{100.0, 50.0, 0.0});
        poly.vertices.push_back(datapod::Point{0.0, 50.0, 0.0});
        poly.vertices.push_back(datapod::Point{0.0, 0.0, 0.0});
        return poly;
    }
} // namespace

TEST_CASE("Farmtrax facade end-to-end") {
    farmtrax::Farmtrax ft;

    datapod::Geo datum{51.0, 5.0, 0.0};
    auto poly = create_test_polygon();

    ft.set_field(poly, datum);
    CHECK(ft.has_field());

    ft.generate_field(10.0, 90.0, 0);
    CHECK(!ft.field()->get_parts().empty());
    CHECK(!ft.field()->get_parts()[0].swaths.empty());

    // Division should produce assignments for machines.
    auto divy = ft.make_divy(farmtrax::DivisionType::ALTERNATE, 2);
    divy.compute_division();
    CHECK(divy.result().swaths_per_machine.size() > 0);

    // Obstacle avoidance should run and return something (may be unchanged).
    datapod::Polygon obstacle;
    obstacle.vertices.push_back(datapod::Point{45.0, 20.0, 0.0});
    obstacle.vertices.push_back(datapod::Point{55.0, 20.0, 0.0});
    obstacle.vertices.push_back(datapod::Point{55.0, 30.0, 0.0});
    obstacle.vertices.push_back(datapod::Point{45.0, 30.0, 0.0});
    obstacle.vertices.push_back(datapod::Point{45.0, 20.0, 0.0});

    auto avoided = ft.avoid_obstacles_for_part({obstacle}, 2.0f, 0);
    CHECK(!avoided.empty());

    // Nety construction through the facade should work.
    auto nety = ft.make_nety_from_part(0);
    CHECK(!nety.get_swaths().empty());

    // Build a drivable tour using turners + headland-following.
    farmtrax::TurnPlannerConfig cfg;
    cfg.swath_width = 10.0;
    cfg.min_turning_radius = 2.0;
    cfg.step_size = 0.2;
    cfg.headland_threshold_rows = 2.0;
    auto tour = ft.build_tour(0, nety.get_swaths(), cfg);
    CHECK(tour.size() >= nety.get_swaths().size());
    bool has_polyline = false;
    for (const auto &s : tour) {
        if (s->type == farmtrax::SwathType::Connection && s->points.size() >= 2) {
            has_polyline = true;
            break;
        }
    }
    CHECK(has_polyline);
}
