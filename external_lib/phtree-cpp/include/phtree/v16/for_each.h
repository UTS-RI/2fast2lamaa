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

#ifndef PHTREE_V16_FOR_EACH_H
#define PHTREE_V16_FOR_EACH_H

#include "phtree/common/common.h"
#include "iterator_with_parent.h"

namespace improbable::phtree::v16 {

/*
 * Iterates over the whole tree. Entries and child nodes that are rejected by the Filter are not
 * traversed or returned.
 */
template <typename T, typename CONVERT, typename CALLBACK_FN, typename FILTER_FN>
class ForEach {
    static constexpr dimension_t DIM = CONVERT::DimInternal;
    using KeyInternal = typename CONVERT::KeyInternal;
    using SCALAR = typename CONVERT::ScalarInternal;
    using EntryT = Entry<DIM, T, SCALAR>;

  public:
    template <typename CB, typename F>
    ForEach(const CONVERT* converter, CB&& callback, F&& filter)
    : converter_{converter}
    , callback_{std::forward<CB>(callback)}
    , filter_(std::forward<F>(filter)) {}

    void Traverse(const EntryT& entry) {
        assert(entry.IsNode());
        auto& entries = entry.GetNode().Entries();
        auto iter = entries.begin();
        auto end = entries.end();
        for (; iter != end; ++iter) {
            const auto& child = iter->second;
            const auto& child_key = child.GetKey();
            if (child.IsNode()) {
                if (filter_.IsNodeValid(child_key, child.GetNodePostfixLen() + 1)) {
                    Traverse(child);
                }
            } else {
                T& value = child.GetValue();
                if (filter_.IsEntryValid(child_key, value)) {
                    callback_(converter_->post(child_key), value);
                }
            }
        }
    }

    const CONVERT* converter_;
    CALLBACK_FN callback_;
    FILTER_FN filter_;
};
}  // namespace improbable::phtree::v16

#endif  // PHTREE_V16_FOR_EACH_H
