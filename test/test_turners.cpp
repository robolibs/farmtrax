#include "doctest/doctest.h"

#include "farmtrax/turners/dubins.hpp"
#include "farmtrax/turners/reeds_shepp.hpp"
#include "farmtrax/turners/sharper.hpp"

TEST_CASE("Turners: Dubins produces a path") {
    farmtrax::turners::Pose2D start(0.0, 0.0, 0.0);
    farmtrax::turners::Pose2D goal(5.0, 2.0, 1.0);

    farmtrax::turners::Dubins dubins(1.0);
    auto path = dubins.plan_path(start, goal, 0.2);

    CHECK(path.total_length > 0.0);
    CHECK(!path.waypoints.empty());
    CHECK(!path.segments.empty());
}

TEST_CASE("Turners: Reeds-Shepp produces a path") {
    farmtrax::turners::Pose2D start(0.0, 0.0, 0.0);
    farmtrax::turners::Pose2D goal(3.0, -2.0, 3.14);

    farmtrax::turners::ReedsShepp rs(1.0);
    auto path = rs.plan_path(start, goal, 0.2);

    CHECK(path.total_length > 0.0);
    CHECK(!path.waypoints.empty());
    CHECK(!path.segments.empty());
}

TEST_CASE("Turners: Sharper produces a turn") {
    farmtrax::turners::Pose2D start(datapod::Point{0.0, 0.0, 0.0}, 0.0);
    farmtrax::turners::Pose2D goal(datapod::Point{0.0, 0.0, 0.0}, 3.14);

    farmtrax::turners::Sharper sharper(1.0, 4.0, 2.0);
    auto path = sharper.plan_sharp_turn(start, goal, "auto");

    CHECK(path.total_length >= 0.0);
    CHECK(!path.waypoints.empty());
    CHECK(path.waypoints.front().point.distance_to(start.point) == doctest::Approx(0.0));
    CHECK(path.waypoints.back().point.distance_to(goal.point) == doctest::Approx(0.0));
}
