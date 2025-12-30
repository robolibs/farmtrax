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
        inline Divy(std::shared_ptr<Field> field, DivisionType type, std::size_t machines)
            : division_type_(type), machine_count_(machines) {
            if (!field)
                throw std::invalid_argument("null field");
            if (machine_count_ == 0)
                throw std::invalid_argument("machine count > 0");
            if (field->get_parts().empty())
                throw std::invalid_argument("field has no parts");
            part_ = std::make_shared<Part>(field->get_parts()[0]);
        }

        inline Divy(std::shared_ptr<Part> part, DivisionType type, std::size_t machines)
            : part_(part), division_type_(type), machine_count_(machines) {
            if (!part)
                throw std::invalid_argument("null field");
            if (machine_count_ == 0)
                throw std::invalid_argument("machine count > 0");
        }

        inline void set_machine_count(std::size_t machines) {
            if (machines == 0)
                throw std::invalid_argument("machine count > 0");
            machine_count_ = machines;
        }

        inline void set_division_type(DivisionType type) { division_type_ = type; }

        inline const DivisionResult &result() const { return division_; }

        inline void compute_division() {
            division_.headlands_per_machine.clear();
            division_.swaths_per_machine.clear();

            std::vector<std::shared_ptr<const Ring>> all_headlands;
            for (auto &h : part_->headlands)
                if (!h.finished)
                    all_headlands.emplace_back(part_, &h);

            std::size_t H = all_headlands.size();
            std::size_t base_h = H / machine_count_;
            std::size_t rem_h = H % machine_count_;
            std::size_t idx_h = 0;
            for (std::size_t m = 0; m < machine_count_; ++m) {
                std::size_t count = base_h + (m < rem_h ? 1 : 0);
                for (std::size_t i = 0; i < count; ++i)
                    division_.headlands_per_machine[m].push_back(all_headlands[idx_h++]);
            }

            std::vector<std::shared_ptr<const Swath>> all_swaths;
            for (auto &s : part_->swaths)
                if (!s.finished && s.type == SwathType::Swath)
                    all_swaths.emplace_back(part_, &s);

            if (division_type_ == DivisionType::SPATIAL_RTREE) {
                SwathRTree rtree;
                for (std::size_t i = 0; i < all_swaths.size(); ++i)
                    rtree.insert(std::make_pair(all_swaths[i]->bounding_box, i));
                std::size_t S = all_swaths.size();
                std::size_t base_s = S / machine_count_;
                std::size_t rem_s = S % machine_count_;
                for (std::size_t m = 0; m < machine_count_; ++m) {
                    std::size_t count = base_s + (m < rem_s ? 1 : 0);
                    std::size_t last_idx = 0;
                    for (std::size_t i = 0; i < count; ++i) {
                        std::size_t idx;
                        if (i == 0) {
                            auto it = rtree.begin();
                            idx = it->second;
                        } else {
                            BBox bb = all_swaths[last_idx]->bounding_box;
                            BPoint center((bb.min_corner().x() + bb.max_corner().x()) / 2.0,
                                          (bb.min_corner().y() + bb.max_corner().y()) / 2.0);
                            std::vector<SwathRTreeValue> result;
                            rtree.query(boost::geometry::index::nearest(center, 1), std::back_inserter(result));
                            idx = result[0].second;
                        }
                        division_.swaths_per_machine[m].push_back(all_swaths[idx]);
                        rtree.remove(std::make_pair(all_swaths[idx]->bounding_box, idx));
                        last_idx = idx;
                    }
                }
            } else if (division_type_ == DivisionType::LENGTH_BALANCED) {
                std::vector<double> loads(machine_count_, 0.0);
                std::vector<std::vector<std::shared_ptr<const Swath>>> assign(machine_count_);
                std::vector<std::pair<double, std::shared_ptr<const Swath>>> by_length;
                for (auto &sw : all_swaths)
                    by_length.emplace_back(sw->line.length(), sw);
                std::sort(by_length.begin(), by_length.end(), [](auto &a, auto &b) { return a.first > b.first; });
                for (auto &p : by_length) {
                    auto it = std::min_element(loads.begin(), loads.end());
                    std::size_t m = std::distance(loads.begin(), it);
                    assign[m].push_back(p.second);
                    loads[m] += p.first;
                }
                for (std::size_t m = 0; m < machine_count_; ++m)
                    division_.swaths_per_machine[m] = std::move(assign[m]);
            } else if (division_type_ == DivisionType::BLOCK) {
                std::size_t S = all_swaths.size();
                std::size_t base_s = S / machine_count_;
                std::size_t rem_s = S % machine_count_;
                std::size_t idx_s = 0;
                for (std::size_t m = 0; m < machine_count_; ++m) {
                    std::size_t count = base_s + (m < rem_s ? 1 : 0);
                    for (std::size_t i = 0; i < count; ++i)
                        division_.swaths_per_machine[m].push_back(all_swaths[idx_s++]);
                }
            } else {
                for (std::size_t i = 0; i < all_swaths.size(); ++i)
                    division_.swaths_per_machine[i % machine_count_].push_back(all_swaths[i]);
            }
        }

      private:
        std::shared_ptr<Part> part_;
        DivisionType division_type_;
        std::size_t machine_count_;
        DivisionResult division_;
    };

} // namespace farmtrax
