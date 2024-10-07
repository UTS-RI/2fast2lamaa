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

#ifndef PHTREE_COMMON_DEBUG_HELPER_H
#define PHTREE_COMMON_DEBUG_HELPER_H

#include "tree_stats.h"

namespace improbable::phtree {

class PhTreeDebugHelper {
  public:
    enum class PrintDetail { name, entries, tree };

    class DebugHelper {
        virtual void CheckConsistency() const = 0;

        [[nodiscard]] virtual PhTreeStats GetStats() const = 0;

        [[nodiscard]] virtual std::string ToString(const PrintDetail& detail) const = 0;
    };

    /*
     * Checks the consistency of the tree. This function requires assertions to be enabled.
     */
    template <typename TREE>
    static void CheckConsistency(const TREE& tree) {
        tree.GetInternalTree().GetDebugHelper().CheckConsistency();
        tree.CheckConsistencyExternal();
    }

    /*
     * Collects some statistics about the tree, such as number of nodes, average depth, ...
     *
     * @return some statistics about the tree.
     */
    template <typename TREE>
    static PhTreeStats GetStats(const TREE& tree) {
        return tree.GetInternalTree().GetDebugHelper().GetStats();
    }

    /*
     * Depending on the detail parameter this returns:
     * - "name"    : a string that identifies the tree implementation type.
     * - "entries" : a string that lists all elements in the tree.
     * - "tree"    : a string that lists all elements in the tree, pretty formatted to indicate tree
     * structure.
     *
     * @return a string as described above.
     */
    template <typename TREE>
    static std::string ToString(const TREE& tree, const PrintDetail& detail) {
        return tree.GetInternalTree().GetDebugHelper().ToString(detail);
    }
};

}  // namespace improbable::phtree
#endif  // PHTREE_COMMON_DEBUG_HELPER_H
