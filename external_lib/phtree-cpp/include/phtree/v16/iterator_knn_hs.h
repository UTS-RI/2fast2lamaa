/*
 * Copyright 2020 Improbable Worlds Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PHTREE_V16_QUERY_KNN_HS_H
#define PHTREE_V16_QUERY_KNN_HS_H

#include "iterator_base.h"
#include "phtree/common/bpt_priority_queue.h"
#include "phtree/common/common.h"
#include <queue>

namespace improbable::phtree::v16 {

/*
 * kNN query implementation that uses preprocessors and distance functions.
 *
 * Implementation (roughly) after Hjaltason and Samet.
 * G. R. Hjaltason and H. Samet., "Distance browsing in spatial databases.", ACM TODS
 * 24(2):265--318. 1999
 */

namespace {
template <dimension_t DIM, typename T, typename SCALAR>
using EntryDist = std::pair<double, const Entry<DIM, T, SCALAR>*>;

template <typename ENTRY>
struct CompareEntryDist {
    bool operator()(const ENTRY& left, const ENTRY& right) const {
        return left.first > right.first;
    };
};
}  // namespace

template <typename T, typename CONVERT, typename DISTANCE, typename FILTER_FN>
class IteratorKnnHS : public IteratorWithFilter<T, CONVERT, FILTER_FN> {
    static constexpr dimension_t DIM = CONVERT::DimInternal;
    using KeyExternal = typename CONVERT::KeyExternal;
    using KeyInternal = typename CONVERT::KeyInternal;
    using SCALAR = typename CONVERT::ScalarInternal;
    using EntryT = typename IteratorWithFilter<T, CONVERT, FILTER_FN>::EntryT;
    using EntryDistT = EntryDist<DIM, T, SCALAR>;

  public:
    template <typename DIST, typename F>
    explicit IteratorKnnHS(
        const EntryT& root,
        size_t min_results,
        const KeyInternal& center,
        const CONVERT* converter,
        DIST&& dist,
        F&& filter)
    : IteratorWithFilter<T, CONVERT, F>(converter, std::forward<F>(filter))
    , center_{center}
    , center_post_{converter->post(center)}
    , current_distance_{std::numeric_limits<double>::max()}
    , remaining_{min_results}
    , queue_n_{}
    , queue_v_{min_results + 1}
    , distance_(std::forward<DIST>(dist)) {
        if (min_results <= 0 || root.GetNode().GetEntryCount() == 0) {
            this->SetFinished();
            return;
        }

        // Initialize queue, use d=0 because every imaginable point lies inside the root Node
        assert(root.IsNode());
        queue_n_.emplace(EntryDistT{0, &root});

        FindNextElement();
    }

    [[nodiscard]] double distance() const {
        return current_distance_;
    }

    IteratorKnnHS& operator++() noexcept {
        FindNextElement();
        return *this;
    }

    [[deprecated]]  // it++ is MUCH slower than ++it!
    IteratorKnnHS
    operator++(int) noexcept {
        IteratorKnnHS iterator(*this);
        ++(*this);
        return iterator;
    }

  private:
    void FindNextElement() {
        while (remaining_ > 0 && !(queue_n_.empty() && queue_v_.empty())) {
            bool use_v = !queue_v_.empty();
            if (use_v && !queue_n_.empty()) {
                use_v = queue_v_.top().first <= queue_n_.top().first;
            }
            if (use_v) {
                // data entry
                auto& result = queue_v_.top();
                --remaining_;
                this->SetCurrentResult(result.second);
                current_distance_ = result.first;
                queue_v_.pop();
                return;
            } else {
                // inner node
                auto top = queue_n_.top();
                auto& node = top.second->GetNode();
                auto d_node = top.first;
                queue_n_.pop();

                if (d_node > max_node_dist_ && queue_v_.size() >= remaining_) {
                    // ignore this node
                    continue;
                }

                for (auto& entry : node.Entries()) {
                    const auto& e2 = entry.second;
                    if (this->ApplyFilter(e2)) {
                        if (e2.IsNode()) {
                            double d = DistanceToNode(e2.GetKey(), e2.GetNodePostfixLen() + 1);
                            if (d <= max_node_dist_) {
                                queue_n_.emplace(d, &e2);
                            }
                        } else {
                            double d = distance_(center_post_, this->post(e2.GetKey()));
                            // Using '<=' allows dealing with infinite distances.
                            if (d <= max_node_dist_) {
                                queue_v_.emplace(d, &e2);
                                if (queue_v_.size() >= remaining_) {
                                    if (queue_v_.size() > remaining_) {
                                        queue_v_.pop_max();
                                    }
                                    double d_max = queue_v_.top_max().first;
                                    max_node_dist_ = std::min(max_node_dist_, d_max);
                                }
                            }
                        }
                    }
                }
            }
        }
        this->SetFinished();
        current_distance_ = std::numeric_limits<double>::max();
    }

    double DistanceToNode(const KeyInternal& prefix, std::uint32_t bits_to_ignore) {
        assert(bits_to_ignore < detail::MAX_BIT_WIDTH<SCALAR>);
        SCALAR mask_min = detail::MAX_MASK<SCALAR> << bits_to_ignore;
        SCALAR mask_max = ~mask_min;
        KeyInternal buf;
        // The following calculates the point inside the node that is closest to center_.
        for (dimension_t i = 0; i < DIM; ++i) {
            // if center_[i] is outside the node, return distance to the closest edge,
            // otherwise return center_[i] itself (assume possible distance=0)
            SCALAR min = prefix[i] & mask_min;
            SCALAR max = prefix[i] | mask_max;
            buf[i] = min > center_[i] ? min : (max < center_[i] ? max : center_[i]);
        }
        return distance_(center_post_, this->post(buf));
    }

  private:
    const KeyInternal center_;
    // center after post processing == the external representation
    const KeyExternal center_post_;
    double current_distance_;
    size_t remaining_;
    std::priority_queue<EntryDistT, std::vector<EntryDistT>, CompareEntryDist<EntryDistT>> queue_n_;
    ::phtree::bptree::detail::priority_queue<EntryDistT, CompareEntryDist<EntryDistT>> queue_v_;
    DISTANCE distance_;
    double max_node_dist_ = std::numeric_limits<double>::infinity();
};

}  // namespace improbable::phtree::v16

#endif  // PHTREE_V16_QUERY_KNN_HS_H
