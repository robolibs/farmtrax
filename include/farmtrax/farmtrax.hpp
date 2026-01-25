#pragma once

#include <memory>
#include <stdexcept>
#include <vector>

#include "farmtrax/avoid.hpp"
#include "farmtrax/divy.hpp"
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp"
#include "farmtrax/turners/dubins.hpp"
#include "farmtrax/turners/reeds_shepp.hpp"
#include "farmtrax/turners/sharper.hpp"

namespace farmtrax {

    /**
     * @brief Facade class that wires together the main Farmtrax modules.
     *
     * This class is intentionally small: it owns (or references) the core objects
     * and provides convenience entry points so users can do most common flows
     * without learning every individual header immediately.
     */
    class Farmtrax {
      public:
        Farmtrax() = default;

        /**
         * @brief Create and store a Field.
         */
        inline void set_field(const datapod::Polygon &border, const datapod::Geo &datum = datapod::Geo{}) {
            field_ = std::make_shared<Field>(border, datum);
        }

        /**
         * @brief Replace the stored field.
         */
        inline void set_field(std::shared_ptr<Field> field) {
            if (!field) {
                throw std::invalid_argument("null field");
            }
            field_ = std::move(field);
        }

        inline bool has_field() const { return static_cast<bool>(field_); }

        inline std::shared_ptr<Field> field() {
            require_field();
            return field_;
        }

        inline std::shared_ptr<const Field> field() const {
            require_field();
            return field_;
        }

        /**
         * @brief Generate headlands and swaths for the currently stored field.
         */
        inline void generate_field(double swath_width, double angle_degrees = 0.0, int headland_count = 1) {
            require_field();
            field_->gen_field(swath_width, angle_degrees, headland_count);
        }

        /**
         * @brief Divide work for a Part (defaults to part 0).
         */
        inline Divy make_divy(DivisionType type, std::size_t machines, std::size_t part_idx = 0) const {
            require_field();
            if (field_->get_parts().empty()) {
                throw std::runtime_error("field has no parts");
            }
            if (part_idx >= field_->get_parts().size()) {
                throw std::out_of_range("part_idx");
            }

            auto part = std::make_shared<Part>(field_->get_parts()[part_idx]);
            return Divy(std::move(part), type, machines);
        }

        /**
         * @brief Build a traversal/graph optimizer from swaths.
         */
        inline Nety make_nety_from_part(std::size_t part_idx = 0) const {
            require_field();
            if (part_idx >= field_->get_parts().size()) {
                throw std::out_of_range("part_idx");
            }

            const auto &swaths = field_->get_parts()[part_idx].swaths;
            std::vector<std::shared_ptr<Swath>> swath_ptrs;
            swath_ptrs.reserve(swaths.size());
            for (const auto &s : swaths) {
                swath_ptrs.emplace_back(std::make_shared<Swath>(s));
            }
            return Nety{swath_ptrs};
        }

        /**
         * @brief Obstacle avoidance over swaths for a Part (defaults to part 0).
         */
        inline std::vector<std::shared_ptr<Swath>>
        avoid_obstacles_for_part(const std::vector<datapod::Polygon> &obstacles, float inflation_distance,
                                 std::size_t part_idx = 0) const {
            require_field();
            if (part_idx >= field_->get_parts().size()) {
                throw std::out_of_range("part_idx");
            }

            const auto &swaths = field_->get_parts()[part_idx].swaths;
            std::vector<std::shared_ptr<const Swath>> swath_ptrs;
            swath_ptrs.reserve(swaths.size());
            for (const auto &s : swaths) {
                swath_ptrs.emplace_back(std::make_shared<Swath>(s));
            }

            ObstacleAvoider avoider(obstacles);
            return avoider.avoid(swath_ptrs, inflation_distance);
        }

        /**
         * @brief Convenience Dubins planner.
         */
        inline turners::DubinsPath plan_dubins(const turners::Pose2D &start, const turners::Pose2D &end,
                                               double min_turning_radius, double step_size = 0.1) const {
            turners::Dubins planner(min_turning_radius);
            return planner.plan_path(start, end, step_size);
        }

        /**
         * @brief Convenience Reeds-Shepp planner.
         */
        inline turners::ReedsSheppPath plan_reeds_shepp(const turners::Pose2D &start, const turners::Pose2D &end,
                                                        double min_turning_radius, double step_size = 0.1) const {
            turners::ReedsShepp planner(min_turning_radius);
            return planner.plan_path(start, end, step_size);
        }

        /**
         * @brief Convenience sharp-turn planner.
         */
        inline turners::Sharper::SharpTurnPath plan_sharp_turn(const turners::Pose2D &start, const turners::Pose2D &end,
                                                               double min_turning_radius, double machine_length,
                                                               double machine_width = 0.0,
                                                               const std::string &pattern = "auto") const {
            turners::Sharper planner(min_turning_radius, machine_length, machine_width);
            return planner.plan_sharp_turn(start, end, pattern);
        }

      private:
        inline void require_field() const {
            if (!field_) {
                throw std::runtime_error("Farmtrax: field not set");
            }
        }

        std::shared_ptr<Field> field_;
    };

} // namespace farmtrax
