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

#ifndef PHTREE_V16_DEBUG_HELPER_H
#define PHTREE_V16_DEBUG_HELPER_H

#include "node.h"
#include "phtree/common/common.h"
#include "phtree/common/debug_helper.h"
#include "phtree_v16.h"
#include <string>

namespace improbable::phtree::v16 {

template <dimension_t DIM, typename T, typename CONVERT>
class PhTreeV16;

template <dimension_t DIM, typename T, typename SCALAR>
class DebugHelperV16 : public PhTreeDebugHelper::DebugHelper {
    using EntryT = Entry<DIM, T, SCALAR>;

  public:
    DebugHelperV16(const EntryT& root, size_t size) : root_{root}, size_{size} {}

    /*
     * Depending on the detail parameter this returns:
     * - "name"    : a string that identifies the tree implementation type.
     * - "entries" : a string that lists all elements in the tree.
     * - "tree"    : a string that lists all elements in the tree, pretty formatted to indicate tree
     * structure.
     *
     * @return a string that identifies the tree implementation type.
     */
    [[nodiscard]] std::string ToString(
        const PhTreeDebugHelper::PrintDetail& detail) const override {
        using Enum = PhTreeDebugHelper::PrintDetail;
        std::ostringstream os;
        switch (detail) {
        case Enum::name:
            os << "PH-TreeV16-C++";
            break;
        case Enum::entries:
            ToStringPlain(os, root_);
            break;
        case Enum::tree:
            ToStringTree(os, 0, root_, detail::MAX_BIT_WIDTH<SCALAR>, true);
            break;
        }
        return os.str();
    }

    /*
     * Collects some statistics about the tree, such as number of nodes, average depth, ...
     *
     * @return some statistics about the tree.
     */
    [[nodiscard]] PhTreeStats GetStats() const override {
        PhTreeStats stats;
        root_.GetNode().GetStats(stats, root_);
        return stats;
    }

    /*
     * Checks the consistency of the tree. This function requires assertions to be enabled.
     */
    void CheckConsistency() const override {
        assert(size_ == root_.GetNode().CheckConsistency(root_));
    }

  private:
    void ToStringPlain(std::ostringstream& os, const EntryT& entry) const {
        for (auto& it : entry.GetNode().Entries()) {
            const auto& child = it.second;
            // inner node?
            if (child.IsNode()) {
                ToStringPlain(os, child);
            } else {
                os << child.GetKey();
                os << "  v=" << (child.IsValue() ? "T" : "null") << std::endl;
            }
        }
    }

    void ToStringTree(
        std::ostringstream& sb,
        detail::bit_width_t current_depth,
        const EntryT& entry,
        const detail::bit_width_t parent_postfix_len,
        bool print_value) const {
        std::string ind = "*";
        for (detail::bit_width_t i = 0; i < current_depth; ++i) {
            ind += "-";
        }
        const auto& node = entry.GetNode();
        const auto infix_len = entry.GetNodeInfixLen(parent_postfix_len);
        const auto postfix_len = entry.GetNodePostfixLen();
        sb << ind << "il=" << infix_len << " pl=" << postfix_len << " ec=" << node.GetEntryCount()
           << " inf=[";

        // for a leaf node, the existence of a sub just indicates that the value exists.
        if (infix_len > 0) {
            detail::bit_mask_t<SCALAR> mask = detail::MAX_MASK<SCALAR> << infix_len;
            mask = ~mask;
            mask <<= (std::uint64_t)postfix_len + 1;
            for (dimension_t i = 0; i < DIM; ++i) {
                sb << detail::ToBinary<SCALAR>(entry.GetKey()[i] & mask) << ",";
            }
        }
        current_depth += infix_len;
        sb << "]  "
           << "Node___il=" << infix_len << ";pl=" << postfix_len
           << ";size=" << node.Entries().size() << std::endl;

        // To clean previous postfixes.
        for (auto& it : node.Entries()) {
            const auto& child = it.second;
            auto hc_pos = it.first;
            if (child.IsNode()) {
                sb << ind << "# " << hc_pos << "  Node: " << std::endl;
                ToStringTree(sb, current_depth + 1, child, postfix_len, print_value);
            } else {
                // post-fix
                sb << ind << detail::ToBinary(child.GetKey());
                sb << "  hcPos=" << hc_pos;
                if (print_value) {
                    sb << "  v=" << (child.IsValue() ? "T" : "null");
                }
                sb << std::endl;
            }
        }
    }

    const EntryT& root_;
    const size_t size_;
};
}  // namespace improbable::phtree::v16

#endif  // PHTREE_V16_DEBUG_HELPER_H
