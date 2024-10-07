/*
 * Copyright 2023 Tilmann Zäschke
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

#ifndef PHTREE_V16_ITERATOR_LOWER_BOUND_H
#define PHTREE_V16_ITERATOR_LOWER_BOUND_H

#include "iterator_base.h"
#include "phtree/common/common.h"

namespace improbable::phtree::v16 {

template <dimension_t DIM, typename T, typename SCALAR>
class Node;

/**
 * This iterator starts at a given position defined by "key" and then iterates until end().
 * @tparam T Value type
 * @tparam CONVERT Converter
 * @tparam FILTER_FN Filter
 */
template <typename T, typename CONVERT, typename FILTER_FN>
class IteratorLowerBound : public IteratorWithFilter<T, CONVERT, FILTER_FN> {
    static constexpr dimension_t DIM = CONVERT::DimInternal;
    using SCALAR = typename CONVERT::ScalarInternal;
    using KeyInternalT = typename CONVERT::KeyInternal;
    using NodeT = Node<DIM, T, SCALAR>;
    using EntryT = typename IteratorWithFilter<T, CONVERT, FILTER_FN>::EntryT;

  public:
    template <typename F>
    IteratorLowerBound(
        const EntryT* root, const KeyInternalT& key, const CONVERT* converter, F&& filter)
    : IteratorWithFilter<T, CONVERT, F>(converter, std::forward<F>(filter))
    , stack_{}
    , stack_size_{0} {
        FindFirstElement(root, key);
    }

    IteratorLowerBound& operator++() noexcept {
        FindNextElement();
        return *this;
    }

    IteratorLowerBound operator++(int) noexcept {
        IteratorFull iterator(*this);
        ++(*this);
        return iterator;
    }

  private:
    void FindFirstElement(const EntryT* root, const KeyInternalT& key) noexcept {
        bool found = false;
        auto* node = &root->GetNode();
        auto it1 = node->LowerBoundC(key, root->GetNodePostfixLen(), found);
        if (it1 != node->End()) {
            Push(it1, node->End());
        }
        if (!found) {
            FindNextElement();
            return;
        }

        while (!IsEmpty()) {
            auto& it = Peek();
            auto& entry = it->second;
            ++it;
            if (entry.IsNode()) {
                found = false;
                auto it2 = entry.GetNode().LowerBoundC(key, entry.GetNodePostfixLen(), found);
                if (it2 != entry.GetNode().End()) {
                    Push(it2, entry.GetNode().End());
                }
                if (!found) {
                    FindNextElement();
                    return;
                }
            } else {
                this->SetCurrentResult(&entry);
                return;
            }
        }
        // finished
        this->SetFinished();
    }

    void FindNextElement() noexcept {
        while (!IsEmpty()) {
            auto* p = &Peek();
            while (*p != PeekEnd()) {
                auto& candidate = (*p)->second;
                ++(*p);
                if (this->ApplyFilter(candidate)) {
                    if (candidate.IsNode()) {
                        p = &PrepareAndPush(candidate.GetNode());
                    } else {
                        this->SetCurrentResult(&candidate);
                        return;
                    }
                }
            }
            // return to parent node
            Pop();
        }
        // finished
        this->SetFinished();
    }

    auto& Push(const EntryIteratorC<DIM, EntryT>& begin, const EntryIteratorC<DIM, EntryT>& end) {
        assert(stack_size_ < stack_.size() - 1);
        stack_[stack_size_].first = begin;
        stack_[stack_size_].second = end;
        ++stack_size_;
        return stack_[stack_size_ - 1].first;
    }

    auto& PrepareAndPush(const NodeT& node) {
        assert(stack_size_ < stack_.size() - 1);
        // No '&'  because this is a temp value
        stack_[stack_size_].first = node.Entries().cbegin();
        stack_[stack_size_].second = node.Entries().end();
        ++stack_size_;
        return stack_[stack_size_ - 1].first;
    }

    auto& Peek() noexcept {
        assert(stack_size_ > 0);
        return stack_[stack_size_ - 1].first;
    }

    auto& PeekEnd() noexcept {
        assert(stack_size_ > 0);
        return stack_[stack_size_ - 1].second;
    }

    auto& Pop() noexcept {
        assert(stack_size_ > 0);
        return stack_[--stack_size_].first;
    }

    bool IsEmpty() noexcept {
        return stack_size_ == 0;
    }

    std::array<
        std::pair<EntryIteratorC<DIM, EntryT>, EntryIteratorC<DIM, EntryT>>,
        detail::MAX_BIT_WIDTH<SCALAR>>
        stack_;
    size_t stack_size_;
};

}  // namespace improbable::phtree::v16

#endif  // PHTREE_V16_ITERATOR_LOWER_BOUND_H
