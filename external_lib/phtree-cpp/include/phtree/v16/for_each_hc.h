/*
 * Copyright 2020 Improbable Worlds Limited
 * Copyright 2022-2023 Tilmann Zäschke
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

#ifndef PHTREE_V16_FOR_EACH_HC_H
#define PHTREE_V16_FOR_EACH_HC_H

#include "iterator_with_parent.h"
#include "phtree/common/common.h"

namespace improbable::phtree::v16 {

/*
 * The HC (hyper cube) iterator uses `hypercube navigation`, ie. filtering of quadrants by their
 * binary hypercube address. In effect it compares the node's volume (box) with the query volume
 * (box) to calculate two bit masks, mask_lower_ and mask_upper_. These can be used as the number of
 * the lowest and highest quadrant that overlaps with the query box. They can also be used to tell
 * for any quadrant whether it overlaps with the query, simply by comparing the quadrant's ID with
 * the two masks, see IsPosValid().
 *
 * For details see  "Efficient Z-Ordered Traversal of Hypercube Indexes" by T. Zäschke, M.C. Norrie,
 * 2017.
 */
template <typename T, typename CONVERT, typename CALLBACK_FN, typename FILTER_FN>
class ForEachHC {
    static constexpr dimension_t DIM = CONVERT::DimInternal;
    using KeyInternal = typename CONVERT::KeyInternal;
    using SCALAR = typename CONVERT::ScalarInternal;
    using EntryT = Entry<DIM, T, SCALAR>;
    using hc_pos_t = detail::hc_pos_dim_t<DIM>;

  public:
    template <typename CB, typename F>
    ForEachHC(
        const KeyInternal& range_min,
        const KeyInternal& range_max,
        const CONVERT* converter,
        CB&& callback,
        F&& filter)
    : min_{range_min}
    , max_{range_max}
    , converter_{converter}
    , callback_{std::forward<CB>(callback)}
    , filter_(std::forward<F>(filter)) {}

    void Traverse(const EntryT& entry, const EntryIteratorC<DIM, EntryT>* opt_it = nullptr) {
        assert(entry.IsNode());
        hc_pos_t mask_lower = 0;
        hc_pos_t mask_upper = 0;
        detail::CalcLimits(entry.GetNodePostfixLen(), min_, max_, entry.GetKey(), mask_lower, mask_upper);
        auto& entries = entry.GetNode().Entries();
        auto postfix_len = entry.GetNodePostfixLen();
        auto end = entries.end();
        auto iter = opt_it != nullptr && *opt_it != end ? *opt_it : entries.lower_bound(mask_lower);
        for (; iter != end && iter->first <= mask_upper; ++iter) {
            auto child_hc_pos = iter->first;
            // Use bit-mask magic to check whether we are in a valid quadrant.
            // -> See paper referenced in class description.
            if (((child_hc_pos | mask_lower) & mask_upper) == child_hc_pos) {
                const auto& child = iter->second;
                const auto& child_key = child.GetKey();
                if (child.IsNode()) {
                    if (CheckNode(child, postfix_len)) {
                        Traverse(child);
                    }
                } else {
                    T& value = child.GetValue();
                    if (detail::IsInRange(child_key, min_, max_) &&
                        filter_.IsEntryValid(child_key, value)) {
                        callback_(converter_->post(child_key), value);
                    }
                }
            }
        }
    }

  private:
    bool CheckNode(const EntryT& entry, detail::bit_width_t parent_postfix_len) {
        const KeyInternal& key = entry.GetKey();
        // Check if the node overlaps with the query box.
        // An infix with len=0 implies that at least part of the child node overlaps with the query,
        // otherwise the bit mask checking would have returned 'false'.
        // Putting it differently, if the infix has len=0, then there is no point in validating it.
        bool mismatch = false;
        if (entry.HasNodeInfix(parent_postfix_len)) {
            // Mask for comparing the prefix with the query boundaries.
            assert(entry.GetNodePostfixLen() + 1 < detail::MAX_BIT_WIDTH<SCALAR>);
            SCALAR comparison_mask = detail::MAX_MASK<SCALAR> << (entry.GetNodePostfixLen() + 1);
            for (dimension_t dim = 0; dim < DIM; ++dim) {
                SCALAR prefix = key[dim] & comparison_mask;
                mismatch |= (prefix > max_[dim] || prefix < (min_[dim] & comparison_mask));
            }
        }
        return !mismatch && filter_.IsNodeValid(key, entry.GetNodePostfixLen() + 1);
    }

    const KeyInternal min_;
    const KeyInternal max_;
    const CONVERT* converter_;
    CALLBACK_FN callback_;
    FILTER_FN filter_;
};
}  // namespace improbable::phtree::v16

#endif  // PHTREE_V16_FOR_EACH_HC_H
