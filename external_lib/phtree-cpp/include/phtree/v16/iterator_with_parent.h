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

#ifndef PHTREE_V16_ITERATOR_SIMPLE_H
#define PHTREE_V16_ITERATOR_SIMPLE_H

#include "phtree/common/common.h"
#include "iterator_base.h"

namespace improbable::phtree::v16 {

template <typename T, typename CONVERT>
class IteratorWithParent : public IteratorWithFilter<T, CONVERT> {
    static constexpr dimension_t DIM = CONVERT::DimInternal;
    using SCALAR = typename CONVERT::ScalarInternal;
    using EntryT = typename IteratorWithFilter<T, CONVERT>::EntryT;
    friend PhTreeV16<DIM, T, CONVERT>;

  public:
    explicit IteratorWithParent(
        const EntryT* current_result,
        const EntryT* current_node,
        const EntryT* parent_node,
        const CONVERT* converter) noexcept
    : IteratorWithFilter<T, CONVERT>(current_result, converter)
    , current_node_{current_node}
    , parent_node_{parent_node} {}

    IteratorWithParent& operator++() {
        this->SetFinished();
        return *this;
    }

    IteratorWithParent operator++(int) {
        IteratorWithParent iterator(*this);
        ++(*this);
        return iterator;
    }

  private:
    /*
     * The parent entry contains the parent node. The parent node is the node ABOVE the current node
     * which contains the current entry.
     */
    EntryT* GetNodeEntry() const {
        return const_cast<EntryT*>(current_node_);
    }

    EntryT* GetParentNodeEntry() const {
        return const_cast<EntryT*>(parent_node_);
    }

    const EntryT* current_node_;
    const EntryT* parent_node_;
};

}  // namespace improbable::phtree::v16

#endif  // PHTREE_V16_ITERATOR_SIMPLE_H
