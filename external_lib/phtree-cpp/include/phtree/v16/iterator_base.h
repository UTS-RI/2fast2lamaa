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

#ifndef PHTREE_V16_ITERATOR_BASE_H
#define PHTREE_V16_ITERATOR_BASE_H

#include "phtree/common/common.h"
#include "phtree/filter.h"
#include "entry.h"

namespace improbable::phtree::v16 {

/*
 * Base class for all PH-Tree iterators.
 */
template <typename EntryT>
class IteratorBase {
  public:
    explicit IteratorBase() noexcept : current_entry_{nullptr} {}
    explicit IteratorBase(const EntryT* current_entry) noexcept : current_entry_{current_entry} {}

    inline auto& operator*() const noexcept {
        assert(current_entry_);
        return current_entry_->GetValue();
    }

    inline auto* operator->() const noexcept {
        assert(current_entry_);
        return &current_entry_->GetValue();
    }

    inline friend bool operator==(
        const IteratorBase<EntryT>& left, const IteratorBase<EntryT>& right) noexcept {
        return left.current_entry_ == right.current_entry_;
    }

    inline friend bool operator!=(
        const IteratorBase<EntryT>& left, const IteratorBase<EntryT>& right) noexcept {
        return left.current_entry_ != right.current_entry_;
    }

    auto& second() const {
        return current_entry_->GetValue();
    }

    [[nodiscard]] inline bool IsEnd() const noexcept {
        return current_entry_ == nullptr;
    }

    inline EntryT* GetEntry() const noexcept {
        return const_cast<EntryT*>(current_entry_);
    }

  protected:
    void SetFinished() {
        current_entry_ = nullptr;
    }

    void SetCurrentResult(const EntryT* current_entry) {
        current_entry_ = current_entry;
    }

  protected:
    const EntryT* current_entry_;
};

template <typename EntryT>
using IteratorEnd = IteratorBase<EntryT>;

template <typename T, typename CONVERT, typename FILTER_FN = FilterNoOp>
class IteratorWithFilter
: public IteratorBase<Entry<CONVERT::DimInternal, T, typename CONVERT::ScalarInternal>> {
  protected:
    static constexpr dimension_t DIM = CONVERT::DimInternal;
    using KeyInternal = typename CONVERT::KeyInternal;
    using SCALAR = typename CONVERT::ScalarInternal;
    using EntryT = Entry<DIM, T, SCALAR>;

  public:
    template <typename F>
    explicit IteratorWithFilter(const CONVERT* converter, F&& filter) noexcept
    : IteratorBase<EntryT>(nullptr), converter_{converter}, filter_{std::forward<F>(filter)} {}

    explicit IteratorWithFilter(const EntryT* current_entry, const CONVERT* converter) noexcept
    : IteratorBase<EntryT>(current_entry), converter_{converter}, filter_{FILTER_FN()} {}

    auto first() const {
        return converter_->post(this->current_entry_->GetKey());
    }

    auto& __Filter() {
        return filter_;
    }

  protected:
    [[nodiscard]] bool ApplyFilter(const EntryT& entry) {
        return entry.IsNode() ? filter_.IsNodeValid(entry.GetKey(), entry.GetNodePostfixLen() + 1)
                              : filter_.IsEntryValid(entry.GetKey(), entry.GetValue());
    }

    auto post(const KeyInternal& point) {
        return converter_->post(point);
    }

  private:
    const CONVERT* converter_;
    FILTER_FN filter_;
};

}  // namespace improbable::phtree::v16

#endif  // PHTREE_V16_ITERATOR_BASE_H
