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

#ifndef PHTREE_V16_ITERATOR_HC_H
#define PHTREE_V16_ITERATOR_HC_H

#include "iterator_with_parent.h"
#include "phtree/common/common.h"

namespace improbable::phtree::v16 {

template <dimension_t DIM, typename T, typename SCALAR>
class Node;

namespace {
template <dimension_t DIM, typename T, typename SCALAR>
class NodeIterator;
}  // namespace

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
template <typename T, typename CONVERT, typename FILTER_FN>
class IteratorHC : public IteratorWithFilter<T, CONVERT, FILTER_FN> {
    static constexpr dimension_t DIM = CONVERT::DimInternal;
    using KeyInternal = typename CONVERT::KeyInternal;
    using SCALAR = typename CONVERT::ScalarInternal;
    using EntryT = typename IteratorWithFilter<T, CONVERT, FILTER_FN>::EntryT;

  public:
    template <typename F>
    IteratorHC(
        const EntryT& root,
        const KeyInternal& range_min,
        const KeyInternal& range_max,
        const CONVERT* converter,
        F&& filter)
    : IteratorWithFilter<T, CONVERT, F>(converter, std::forward<F>(filter))
    , stack_size_{0}
    , min_{range_min}
    , max_{range_max} {
        stack_.reserve(8);
        PrepareAndPush(root);
        FindNextElement();
    }

    IteratorHC& operator++() noexcept {
        FindNextElement();
        return *this;
    }

    IteratorHC operator++(int) noexcept {
        IteratorHC iterator(*this);
        ++(*this);
        return iterator;
    }

  private:
    void FindNextElement() noexcept {
        while (!IsEmpty()) {
            auto* p = &Peek();
            const EntryT* current_result;
            while ((current_result = p->Increment(min_, max_))) {
                if (this->ApplyFilter(*current_result)) {
                    if (current_result->IsNode()) {
                        p = &PrepareAndPush(*current_result);
                    } else {
                        this->SetCurrentResult(current_result);
                        return;
                    }
                }
            }
            // no matching (more) elements found
            Pop();
        }
        // finished
        this->SetFinished();
    }

    auto& PrepareAndPush(const EntryT& entry) noexcept {
        if (stack_.size() < stack_size_ + 1) {
            stack_.emplace_back();
        }
        assert(stack_size_ < stack_.size());
        auto& ni = stack_[stack_size_++];
        ni.Init(min_, max_, entry);
        return ni;
    }

    auto& Peek() noexcept {
        assert(stack_size_ > 0);
        return stack_[stack_size_ - 1];
    }

    auto& Pop() noexcept {
        assert(stack_size_ > 0);
        return stack_[--stack_size_];
    }

    bool IsEmpty() noexcept {
        return stack_size_ == 0;
    }

    std::vector<NodeIterator<DIM, T, SCALAR>> stack_;
    size_t stack_size_;
    const KeyInternal min_;
    const KeyInternal max_;
};

namespace {
template <dimension_t DIM, typename T, typename SCALAR>
class NodeIterator {
    using KeyT = PhPoint<DIM, SCALAR>;
    using EntryT = Entry<DIM, T, SCALAR>;
    using EntriesT = const EntryMap<DIM, EntryT>;
    using hc_pos_t = detail::hc_pos_dim_t<DIM>;

  public:
    NodeIterator() : iter_{}, entries_{nullptr}, mask_lower_{0}, mask_upper_{0}, postfix_len_{0} {}

    void Init(const KeyT& min, const KeyT& max, const EntryT& entry) {
        auto& node = entry.GetNode();
        detail::CalcLimits(
            entry.GetNodePostfixLen(), min, max, entry.GetKey(), mask_lower_, mask_upper_);
        iter_ = node.Entries().lower_bound(mask_lower_);
        entries_ = &node.Entries();
        postfix_len_ = entry.GetNodePostfixLen();
    }

    /*
     * Advances the cursor.
     * @return TRUE iff a matching element was found.
     */
    const EntryT* Increment(const KeyT& range_min, const KeyT& range_max) {
        while (iter_ != entries_->end() && iter_->first <= mask_upper_) {
            if (IsPosValid(iter_->first)) {
                const auto* be = &iter_->second;
                if (CheckEntry(*be, range_min, range_max)) {
                    ++iter_;
                    return be;
                }
            }
            ++iter_;
        }
        return nullptr;
    }

    bool CheckEntry(const EntryT& candidate, const KeyT& range_min, const KeyT& range_max) const {
        if (candidate.IsValue()) {
            return detail::IsInRange(candidate.GetKey(), range_min, range_max);
        }

        // Check if node-prefix allows sub-node to contain any useful values.
        // An infix with len=0 implies that at least part of the child node overlaps with the query.
        // Putting it differently, if the infix has len=0, then there is no point in validating it.
        if (!candidate.HasNodeInfix(postfix_len_)) {
            return true;
        }

        // Mask for comparing the prefix with the query boundaries.
        assert(candidate.GetNodePostfixLen() + 1 < detail::MAX_BIT_WIDTH<SCALAR>);
        SCALAR comparison_mask = detail::MAX_MASK<SCALAR> << (candidate.GetNodePostfixLen() + 1);
        auto& key = candidate.GetKey();
        for (dimension_t dim = 0; dim < DIM; ++dim) {
            SCALAR in = key[dim] & comparison_mask;
            if (in > range_max[dim] || in < (range_min[dim] & comparison_mask)) {
                return false;
            }
        }
        return true;
    }

  private:
    [[nodiscard]] inline bool IsPosValid(hc_pos_t key) const noexcept {
        return ((key | mask_lower_) & mask_upper_) == key;
    }

  private:
    EntryIteratorC<DIM, EntryT> iter_;
    EntriesT* entries_;
    hc_pos_t mask_lower_;
    hc_pos_t mask_upper_;
    detail::bit_width_t postfix_len_;
};
}  // namespace
}  // namespace improbable::phtree::v16

#endif  // PHTREE_V16_ITERATOR_HC_H
