#pragma once
#include "farmtrax/field.hpp"
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace farmtrax {

    enum class DivisionType { BLOCK, ALTERNATE, SPATIAL_RTREE, LENGTH_BALANCED };

    struct DivisionResult {
        std::unordered_map<std::size_t, std::vector<std::shared_ptr<const Ring>>> headlands_per_machine;
        std::unordered_map<std::size_t, std::vector<std::shared_ptr<const Swath>>> swaths_per_machine;
    };

    class Divy {
      public:
        Divy(std::shared_ptr<Field> field, DivisionType type, std::size_t machines);
        Divy(std::shared_ptr<Part> part, DivisionType type, std::size_t machines);
        void set_machine_count(std::size_t machines);
        void set_division_type(DivisionType type);
        const DivisionResult &result() const;
        void compute_division();

      private:
        std::shared_ptr<Part> part_;
        DivisionType division_type_;
        std::size_t machine_count_;
        DivisionResult division_;
    };

} // namespace farmtrax
