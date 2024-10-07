/*
 * Copyright 2022 Tilmann Zäschke
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

#ifndef PHTREE_COMMON_B_PLUS_TREE_H
#define PHTREE_COMMON_B_PLUS_TREE_H

#include "b_plus_tree_base.h"
#include "bits.h"
#include <cassert>
#include <tuple>
#include <vector>

/*
 * PLEASE do not include this file directly, it is included via common.h.
 *
 * This file contains the B+tree implementation which is used in high-dimensional nodes in
 * the PH-Tree.
 */
namespace phtree::bptree {

/*
 * The b_plus_tree_map is a B+tree implementation that uses a hierarchy of horizontally
 * connected nodes for fast traversal through all entries.
 *
 * Behavior:
 * This is a key-value map. Keys are unique, so for every key there is at most one entry.
 *
 * The individual nodes have at most M entries.
 * The tree has O(log n) lookup and O(M log n) insertion/removal time complexity,
 * space complexity is O(n).
 *
 * Tree structure:
 * - Inner nodes: have other nodes as children; their key of an entry represents the highest
 *   key of any subnode in that entry
 * - Leaf nodes: have values as children; their key represents the key of a key/value pair
 * - Every node is either a leaf (l-node; contains values) or an inner node
 *   (n-node; contains nodes).
 * - "Sibling" nodes refer to the nodes linked by prev_node_ or next_node_. Sibling nodes
 *   usually have the same parent but may also be children of their parent's siblings.
 *
 * - Guarantee: All leaf nodes are horizontally connected
 * - Inner nodes may or may not be connected. Specifically:
 *   - New inner nodes will be assigned siblings from the same parent or the parent's sibling
 *     (if the new node is the first or last node in a parent)
 *   - There is no guarantee that inner nodes know about their potential sibling (=other inner
 *     nodes that own bordering values/child-nodes).
 *   - There is no guarantee that siblings are on the same depth of the tree.
 * - The tree is not balanced
 *
 * TODO since this is a "map" (with 1:1 mapping of key:value), we could optimize splitting and
 *   merging by trying to reduce `dead space`
 *   (space between key1 and key2 that exceeds (key2 - key1)).
 */
template <typename KeyT, typename ValueT, size_t COUNT_MAX>
class b_plus_tree_map {
    static_assert(std::is_integral<KeyT>() && "Key type must be integer");
    static_assert(std::is_unsigned<KeyT>() && "Key type must unsigned");

    // COUNT_MAX indicates that a tree will never have to hold more than COUNT_MAX entries.
    // We can use this to optimize node sizes for small trees.
    constexpr static size_t LEAF_MAX = std::min(size_t(16), COUNT_MAX);
    // Special case for small COUNT with smaller inner leaf or
    // trees with a single inner leaf. '*2' is added because leaf filling is not compact.
    constexpr static size_t INNER_MAX = std::min(size_t(16), COUNT_MAX / LEAF_MAX * 2);
    static_assert(LEAF_MAX > 2 && LEAF_MAX < 1000);
    static_assert(COUNT_MAX <= (16 * 16) || (INNER_MAX > 2 && INNER_MAX < 1000));
    // TODO This could be improved but requires a code change to move > 1 entry when merging.
    constexpr static size_t LEAF_MIN = 2;   // std::max((size_t)2, M_leaf >> 2);
    constexpr static size_t INNER_MIN = 2;  // std::max((size_t)2, M_inner >> 2);
    constexpr static size_t LEAF_INIT = std::min(size_t(2), LEAF_MAX);
    constexpr static size_t INNER_INIT = std::min(size_t(4), INNER_MAX);
    using LEAF_CFG = detail::bpt_config<LEAF_MAX, LEAF_MIN, LEAF_INIT>;
    using INNER_CFG = detail::bpt_config<INNER_MAX, INNER_MIN, INNER_INIT>;

    class bpt_node_leaf;
    class bpt_iterator;
    using IterT = bpt_iterator;
    using NLeafT = bpt_node_leaf;
    using NInnerT = detail::bpt_node_inner<KeyT, NLeafT, INNER_CFG>;
    using NodeT = detail::bpt_node_base<KeyT, NInnerT, bpt_node_leaf>;
    using TreeT = b_plus_tree_map<KeyT, ValueT, COUNT_MAX>;

  public:
    explicit b_plus_tree_map() : root_{new NLeafT(nullptr, nullptr, nullptr)}, size_{0} {};

    b_plus_tree_map(const b_plus_tree_map& other) = delete;
    b_plus_tree_map& operator=(const b_plus_tree_map& other) = delete;

    b_plus_tree_map(b_plus_tree_map&& other) noexcept : root_{other.root_}, size_{other.size_} {
        other.root_ = nullptr;
        other.size_ = 0;
    }

    b_plus_tree_map& operator=(b_plus_tree_map&& other) noexcept {
        delete root_;
        root_ = other.root_;
        other.root_ = nullptr;
        size_ = other.size_;
        other.size_ = 0;
        return *this;
    }

    ~b_plus_tree_map() noexcept {
        delete root_;
    }

    [[nodiscard]] auto find(KeyT key) noexcept {
        auto leaf = lower_bound_leaf(key, root_);
        return leaf != nullptr ? leaf->find(key) : IterT{};
    }

    [[nodiscard]] auto find(KeyT key) const noexcept {
        return const_cast<b_plus_tree_map&>(*this).find(key);
    }

    [[nodiscard]] auto lower_bound(KeyT key) noexcept {
        auto leaf = lower_bound_leaf(key, root_);
        return leaf != nullptr ? leaf->template lower_bound_as_iter<IterT>(key) : IterT{};
    }

    [[nodiscard]] auto lower_bound(KeyT key) const noexcept {
        return const_cast<b_plus_tree_map&>(*this).lower_bound(key);
    }

    [[nodiscard]] auto begin() noexcept {
        return IterT(root_);
    }

    [[nodiscard]] auto begin() const noexcept {
        return IterT(root_);
    }

    [[nodiscard]] auto cbegin() const noexcept {
        return IterT(root_);
    }

    [[nodiscard]] auto end() noexcept {
        return IterT();
    }

    [[nodiscard]] auto end() const noexcept {
        return IterT();
    }

    [[nodiscard]] auto cend() const noexcept {
        return IterT();
    }

    template <typename... Args>
    auto emplace(Args&&... args) {
        return try_emplace(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto emplace_hint(const IterT& hint, KeyT key, Args&&... args) {
        if (empty() || hint.is_end()) {
            return emplace(key, std::forward<Args>(args)...);
        }
        assert(hint.node_->is_leaf());

        auto node = hint.node_->as_leaf();

        // The following may drop a valid hint but is easy to check.
        if (node->data_.begin()->first > key || (node->data_.end() - 1)->first < key) {
            return emplace(key, std::forward<Args>(args)...);
        }
        return node->try_emplace(key, root_, size_, std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto try_emplace(KeyT key, Args&&... args) {
        auto leaf = lower_bound_or_last_leaf(key, root_);
        return leaf->try_emplace(key, root_, size_, std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto try_emplace(IterT iter, KeyT key, Args&&... args) {
        return emplace_hint(iter, key, std::forward<Args>(args)...).first;
    }

    void erase(KeyT key) {
        auto leaf = lower_bound_leaf(key, root_);
        if (leaf != nullptr) {
            size_ -= leaf->erase_key(key, root_);
        }
    }

    void erase(const IterT& iterator) {
        assert(iterator != end());
        --size_;
        iterator.node_->erase_entry(iterator.iter_, root_);
    }

    [[nodiscard]] size_t size() const noexcept {
        return size_;
    }

    [[nodiscard]] bool empty() const noexcept {
        return size_ == 0;
    }

    void _check() const {
        size_t count = 0;
        const NLeafT* prev_leaf = nullptr;
        KeyT known_min = std::numeric_limits<KeyT>::max();
        root_->_check(count, nullptr, prev_leaf, known_min, 0);
        assert(count == size());
    }

  private:
    using bpt_leaf_super =
        ::phtree::bptree::detail::bpt_node_data<KeyT, ValueT, NInnerT, NLeafT, true, LEAF_CFG>;
    class bpt_node_leaf : public bpt_leaf_super {
      public:
        explicit bpt_node_leaf(NInnerT* parent, NLeafT* prev, NLeafT* next) noexcept
        : bpt_leaf_super(true, parent, prev, next) {}

        ~bpt_node_leaf() noexcept = default;

        [[nodiscard]] IterT find(KeyT key) noexcept {
            auto it = this->lower_bound(key);
            if (it != this->data_.end() && it->first == key) {
                return IterT(this, it);
            }
            return IterT();
        }

        template <typename... Args>
        auto try_emplace(KeyT key, NodeT*& root, size_t& entry_count, Args&&... args) {
            auto it = this->lower_bound(key);
            if (it != this->data_.end() && it->first == key) {
                return std::make_pair(IterT(this, it), false);
            }
            ++entry_count;

            auto full_it = this->template check_split_and_adjust_iterator<IterT>(it, key, root);
            auto it_result = full_it.node_->data_.emplace(
                full_it.iter_,
                std::piecewise_construct,
                std::forward_as_tuple(key),
                std::forward_as_tuple(std::forward<Args>(args)...));
            return std::make_pair(IterT(full_it.node_, it_result), true);
        }

        bool erase_key(KeyT key, NodeT*& root) {
            auto it = this->lower_bound(key);
            if (it != this->data_.end() && it->first == key) {
                this->erase_entry(it, root);
                return true;
            }
            return false;
        }

        void _check(
            size_t& count,
            const NInnerT* parent,
            const NLeafT*& prev_leaf,
            KeyT& known_min,
            KeyT known_max) const {
            this->_check_data(parent, known_max);

            assert(prev_leaf == this->prev_node_);
            for (auto& e : this->data_) {
                assert(count == 0 || e.first > known_min);
                assert(this->parent_ == nullptr || e.first <= known_max);
                ++count;
                known_min = e.first;
            }
            prev_leaf = this;
        }
    };

    class bpt_iterator : public detail::bpt_iterator_base<NLeafT, NodeT, TreeT> {
        using SuperT = detail::bpt_iterator_base<NLeafT, NodeT, TreeT>;

      public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = ValueT;
        using difference_type = std::ptrdiff_t;
        using pointer = ValueT*;
        using reference = ValueT&;

        // Arbitrary position iterator
        explicit bpt_iterator(NLeafT* node, typename SuperT::LeafIteratorT it) noexcept
        : SuperT(node, it) {}

        // begin() iterator
        explicit bpt_iterator(NodeT* node) noexcept : SuperT(node) {}

        // end() iterator
        bpt_iterator() noexcept : SuperT() {}

        auto& operator*() const noexcept {
            return *this->iter();
        }

        auto* operator->() const noexcept {
            return &*this->iter();
        }
    };

  private:
    NodeT* root_;
    size_t size_;
};
}  // namespace phtree::bptree

#endif  // PHTREE_COMMON_B_PLUS_TREE_H
