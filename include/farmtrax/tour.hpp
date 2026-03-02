#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "farmtrax/field.hpp"
#include "farmtrax/turners/dubins.hpp"
#include "farmtrax/turners/reeds_shepp.hpp"
#include "farmtrax/turners/sharper.hpp"
#include "farmtrax/utils/utils.hpp"

namespace farmtrax {

    struct TurnPlannerConfig {
        enum class Model {
            Auto,
            Dubins,
            ReedsShepp,
            Sharper,
        };

        Model model = Model::Auto;
        double min_turning_radius = 2.0;
        double step_size = 0.2;

        // Used by Sharper
        double machine_length = 6.0;
        double machine_width = 0.0;
        std::string sharper_pattern = "auto";

        // Used by "far gap" logic
        double swath_width = 0.0;
        double headland_threshold_rows = 2.0;
    };

    class TourBuilder {
      public:
        /**
         * @brief Build a drivable tour by inserting connector segments.
         *
         * Connectors are either:
         * - direct turn (turners)
         * - turn -> follow headland polyline -> turn, when endpoints are farther than
         *   `headland_threshold_rows` swath widths apart.
         */
        static inline std::vector<std::shared_ptr<const Swath>>
        build(const Part &part, const std::vector<std::shared_ptr<const Swath>> &ordered_swaths,
              const TurnPlannerConfig &cfg) {
            if (ordered_swaths.empty()) {
                return {};
            }

            auto ring = select_headland_ring(part);
            if (ring.vertices.size() < 2) {
                throw std::runtime_error("TourBuilder: invalid headland ring");
            }

            std::vector<std::shared_ptr<const Swath>> out;
            out.reserve(ordered_swaths.size() * 2);

            for (std::size_t i = 0; i < ordered_swaths.size(); ++i) {
                const auto &cur = ordered_swaths[i];
                out.push_back(cur);

                if (i + 1 >= ordered_swaths.size()) {
                    break;
                }

                const auto &nxt = ordered_swaths[i + 1];
                auto connectors = connect_between_swaths(*cur, *nxt, ring, cfg);
                out.insert(out.end(), connectors.begin(), connectors.end());
            }

            return out;
        }

      private:
        static inline datapod::Polygon select_headland_ring(const Part &part) {
            // Prefer the outermost headland (closest to boundary). If no headlands exist,
            // fall back to the part boundary.
            if (!part.headlands.empty()) {
                return part.headlands.front().polygon;
            }
            return part.boundary.polygon;
        }

        static inline double safe_swath_width(const Swath &a, const Swath &b, const TurnPlannerConfig &cfg) {
            if (cfg.swath_width > 0.0) {
                return cfg.swath_width;
            }
            if (a.width > 0.0) {
                return a.width;
            }
            if (b.width > 0.0) {
                return b.width;
            }
            return 0.0;
        }

        static inline double lateral_rows_between(const Swath &from, const Swath &to, const datapod::Point &from_end,
                                                  const datapod::Point &to_start, double swath_width) {
            if (swath_width <= 0.0) {
                // Fallback: if we don't know row spacing, treat as always "close".
                return 0.0;
            }

            // Compute lateral offset relative to the "from" swath direction.
            auto head = from.getHead();
            auto tail = from.getTail();
            double dx = tail.x - head.x;
            double dy = tail.y - head.y;
            double len = std::sqrt(dx * dx + dy * dy);
            if (len < 1e-9) {
                return 0.0;
            }

            dx /= len;
            dy /= len;

            // Left normal.
            double nx = -dy;
            double ny = dx;

            double vx = to_start.x - from_end.x;
            double vy = to_start.y - from_end.y;
            double lateral = std::abs(vx * nx + vy * ny);

            return lateral / swath_width;
        }

        struct Projection {
            datapod::Point point;
            std::size_t seg_idx = 0;
            double t = 0.0; // [0,1] along segment
        };

        static inline Projection project_to_polyline(const datapod::Polygon &ring, const datapod::Point &p) {
            if (ring.vertices.size() < 2) {
                throw std::runtime_error("TourBuilder: ring too small");
            }

            double best_d2 = std::numeric_limits<double>::infinity();
            Projection best;

            auto clamp01 = [](double v) { return std::max(0.0, std::min(1.0, v)); };

            const auto &v = ring.vertices;
            for (std::size_t i = 0; i + 1 < v.size(); ++i) {
                const auto &a = v[i];
                const auto &b = v[i + 1];
                double abx = b.x - a.x;
                double aby = b.y - a.y;
                double apx = p.x - a.x;
                double apy = p.y - a.y;
                double denom = abx * abx + aby * aby;
                double t = (denom > 1e-12) ? (apx * abx + apy * aby) / denom : 0.0;
                t = clamp01(t);

                datapod::Point q{a.x + t * abx, a.y + t * aby, a.z + t * (b.z - a.z)};
                double dx = p.x - q.x;
                double dy = p.y - q.y;
                double dz = p.z - q.z;
                double d2 = dx * dx + dy * dy + dz * dz;
                if (d2 < best_d2) {
                    best_d2 = d2;
                    best.point = q;
                    best.seg_idx = i;
                    best.t = t;
                }
            }

            return best;
        }

        static inline datapod::Vector<datapod::Point> path_along_ring(const datapod::Polygon &ring, const Projection &a,
                                                                      const Projection &b, bool forward) {
            // Assumes ring.vertices is a polyline where consecutive points form edges.
            // We output an L-shaped (or more) path by walking vertex-to-vertex (no shortcut).
            datapod::Vector<datapod::Point> out;
            const auto &v = ring.vertices;
            std::size_t nseg = (v.size() >= 2) ? (v.size() - 1) : 0;
            if (nseg == 0) {
                return out;
            }

            auto next_seg = [&](std::size_t idx) { return (idx + 1) % nseg; };
            auto prev_seg = [&](std::size_t idx) { return (idx + nseg - 1) % nseg; };

            out.push_back(a.point);

            if (a.seg_idx == b.seg_idx) {
                if ((forward && a.t <= b.t) || (!forward && a.t >= b.t)) {
                    out.push_back(b.point);
                    return out;
                }
                // Same segment but wrong direction: wrap around whole ring.
            }

            if (forward) {
                // Move to end of a segment, then walk segment ends until we reach b.
                out.push_back(v[a.seg_idx + 1]);

                std::size_t idx = next_seg(a.seg_idx);
                while (idx != b.seg_idx) {
                    out.push_back(v[idx + 1]);
                    idx = next_seg(idx);
                }

                out.push_back(b.point);
                return out;
            }

            // Backward: go to start vertex of a.seg_idx, then walk backwards.
            out.push_back(v[a.seg_idx]);
            std::size_t idx = prev_seg(a.seg_idx);
            while (idx != b.seg_idx) {
                out.push_back(v[idx]);
                idx = prev_seg(idx);
            }
            out.push_back(b.point);
            return out;
        }

        static inline double polyline_length(const datapod::Vector<datapod::Point> &pts) {
            if (pts.size() < 2) {
                return 0.0;
            }
            double len = 0.0;
            for (std::size_t i = 1; i < pts.size(); ++i) {
                len += pts[i - 1].distance_to(pts[i]);
            }
            return len;
        }

        static inline datapod::AABB aabb_from_points(const datapod::Vector<datapod::Point> &pts) {
            if (pts.empty()) {
                return datapod::AABB{};
            }
            datapod::Point mn = pts[0];
            datapod::Point mx = pts[0];
            for (const auto &p : pts) {
                mn.x = std::min(mn.x, p.x);
                mn.y = std::min(mn.y, p.y);
                mn.z = std::min(mn.z, p.z);
                mx.x = std::max(mx.x, p.x);
                mx.y = std::max(mx.y, p.y);
                mx.z = std::max(mx.z, p.z);
            }
            return datapod::AABB{mn, mx};
        }

        static inline std::shared_ptr<Swath> make_polyline_swath(const datapod::Vector<datapod::Point> &pts,
                                                                 SwathType type) {
            if (pts.size() < 2) {
                return nullptr;
            }
            auto sw = std::make_shared<Swath>(create_swath(pts.front(), pts.back(), type));
            sw->points = pts;
            sw->bounding_box = aabb_from_points(pts);
            return sw;
        }

        static inline datapod::Vector<datapod::Point>
        plan_turn_points(const turners::Pose2D &start, const turners::Pose2D &goal, const TurnPlannerConfig &cfg) {
            TurnPlannerConfig::Model model = cfg.model;
            if (model == TurnPlannerConfig::Model::Auto) {
                // Simple auto rule:
                // - if the poses are nearly coincident, prefer Sharper
                // - else use Reeds-Shepp (supports reverse), fallback to Dubins if needed
                double d = start.point.distance_to(goal.point);
                if (d < cfg.min_turning_radius * 0.25) {
                    model = TurnPlannerConfig::Model::Sharper;
                } else {
                    model = TurnPlannerConfig::Model::ReedsShepp;
                }
            }

            if (model == TurnPlannerConfig::Model::Dubins) {
                turners::Dubins planner(cfg.min_turning_radius);
                auto path = planner.plan_path(start, goal, cfg.step_size);
                datapod::Vector<datapod::Point> pts;
                pts.reserve(path.waypoints.size());
                for (const auto &wp : path.waypoints) {
                    pts.push_back(wp.point);
                }
                if (pts.empty()) {
                    pts.push_back(start.point);
                    pts.push_back(goal.point);
                }
                return pts;
            }

            if (model == TurnPlannerConfig::Model::Sharper) {
                turners::Sharper planner(cfg.min_turning_radius, cfg.machine_length, cfg.machine_width);
                auto path = planner.plan_sharp_turn(start, goal, cfg.sharper_pattern);
                datapod::Vector<datapod::Point> pts;
                pts.reserve(path.waypoints.size());
                for (const auto &wp : path.waypoints) {
                    pts.push_back(wp.point);
                }
                if (pts.empty()) {
                    pts.push_back(start.point);
                    pts.push_back(goal.point);
                }
                return pts;
            }

            // Default to Reeds-Shepp
            turners::ReedsShepp planner(cfg.min_turning_radius);
            auto path = planner.plan_path(start, goal, cfg.step_size);
            datapod::Vector<datapod::Point> pts;
            pts.reserve(path.waypoints.size());
            for (const auto &wp : path.waypoints) {
                pts.push_back(wp.point);
            }
            if (pts.empty()) {
                pts.push_back(start.point);
                pts.push_back(goal.point);
            }
            return pts;
        }

        static inline std::pair<double, double> swath_headings_for_directed_pair(const Swath &from, const Swath &to,
                                                                                 const datapod::Point &to_start) {
            // from is already directed: head->tail.
            double start_yaw = utils::heading_between(from.getHead(), from.getTail());

            // Infer desired entry heading for "to" given the chosen start endpoint.
            datapod::Point to_other = utils::points_equal(to_start, to.getHead()) ? to.getTail() : to.getHead();
            double end_yaw = utils::heading_between(to_start, to_other);

            return {start_yaw, end_yaw};
        }

        static inline std::vector<std::shared_ptr<const Swath>>
        connect_between_swaths(const Swath &from, const Swath &to, const datapod::Polygon &headland_ring,
                               const TurnPlannerConfig &cfg) {
            // from and to are assumed already directed in traversal order.
            datapod::Point from_end = from.getTail();
            datapod::Point to_start = to.getHead();

            double swath_width = safe_swath_width(from, to, cfg);
            double rows = lateral_rows_between(from, to, from_end, to_start, swath_width);

            if (rows <= cfg.headland_threshold_rows) {
                auto [start_yaw, end_yaw] = swath_headings_for_directed_pair(from, to, to_start);
                turners::Pose2D start(from_end, start_yaw);
                turners::Pose2D goal(to_start, end_yaw);
                auto pts = plan_turn_points(start, goal, cfg);
                auto sw = make_polyline_swath(pts, SwathType::Connection);
                if (!sw) {
                    return {};
                }
                return {std::static_pointer_cast<const Swath>(sw)};
            }

            // Far: go turn -> follow headland exactly (vertex walk) -> turn.
            auto proj_a = project_to_polyline(headland_ring, from_end);
            auto proj_b = project_to_polyline(headland_ring, to_start);

            auto ring_fwd = path_along_ring(headland_ring, proj_a, proj_b, true);
            auto ring_bwd = path_along_ring(headland_ring, proj_a, proj_b, false);
            bool use_fwd = polyline_length(ring_fwd) <= polyline_length(ring_bwd);
            auto ring_path = use_fwd ? ring_fwd : ring_bwd;

            std::vector<std::shared_ptr<const Swath>> out;
            out.reserve(3);

            // Turn from swath end to ring entry.
            {
                // Heading at ring entry: approximate as direction of first ring segment.
                datapod::Point ring_next = (ring_path.size() >= 2) ? ring_path[1] : proj_a.point;
                double ring_yaw = utils::heading_between(proj_a.point, ring_next);
                double start_yaw = utils::heading_between(from.getHead(), from.getTail());

                turners::Pose2D start(from_end, start_yaw);
                turners::Pose2D goal(proj_a.point, ring_yaw);
                auto pts = plan_turn_points(start, goal, cfg);
                if (auto sw = make_polyline_swath(pts, SwathType::Connection)) {
                    out.push_back(std::static_pointer_cast<const Swath>(sw));
                }
            }

            // Headland travel segment (exact polyline along ring).
            if (auto sw = make_polyline_swath(ring_path, SwathType::Connection)) {
                out.push_back(std::static_pointer_cast<const Swath>(sw));
            }

            // Turn from ring exit into next swath start.
            {
                datapod::Point ring_prev = (ring_path.size() >= 2) ? ring_path[ring_path.size() - 2] : proj_b.point;
                double ring_yaw = utils::heading_between(ring_prev, proj_b.point);
                auto [start_yaw_ignored, end_yaw] = swath_headings_for_directed_pair(from, to, to_start);

                turners::Pose2D start(proj_b.point, ring_yaw);
                turners::Pose2D goal(to_start, end_yaw);
                auto pts = plan_turn_points(start, goal, cfg);
                if (auto sw = make_polyline_swath(pts, SwathType::Connection)) {
                    out.push_back(std::static_pointer_cast<const Swath>(sw));
                }
            }

            return out;
        }
    };

} // namespace farmtrax
