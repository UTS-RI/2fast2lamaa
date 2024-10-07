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

#ifndef PHTREE_COMMON_B_PLUS_TREE_MULTIMAP_H
#define PHTREE_COMMON_B_PLUS_TREE_MULTIMAP_H

#include "b_plus_tree_base.h"
#include "bits.h"
#include <cassert>
#include <tuple>
#include <vector>

/*
 * PLEASE do not include this file directly, it is included via common.h.
 *
 * This file contains the B+tree multimap implementation which is used in high-dimensional nodes in
 * the PH-Tree.
 */
namespace phtree::bptree {

/*
 * The b_plus_tree_multimap is a B+tree implementation that uses a hierarchy of horizontally
 * connected nodes for fast traversal through all entries.
 *
 * Behavior
 * ========
 * This is a multimap. It behaves just like std::multimap, minus some API functions.
 * The set/map is ordered by their key.  Entries with identical keys have no specific ordering
 * but the order is stable with respect to insertion/removal of other entries.
 *
 *
 * Rationale
 * =========
 * This implementations is optimized for small entry count, however it should
 * scale well with large entry counts.
 *
 *
 * Internals
 * =========
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
 */
template <typename KeyT, typename ValueT>
class b_plus_tree_multimap {
    static_assert(std::is_integral<KeyT>() && "Key type must be integer");
    static_assert(std::is_unsigned<KeyT>() && "Key type must unsigned");

    class bpt_node_leaf;
    class bpt_iterator;
    using IterT = bpt_iterator;
    using NLeafT = bpt_node_leaf;
    using NInnerT = detail::bpt_node_inner<KeyT, NLeafT>;
    using NodeT = detail::bpt_node_base<KeyT, NInnerT, bpt_node_leaf>;
    using TreeT = b_plus_tree_multimap<KeyT, ValueT>;

  public:
    explicit b_plus_tree_multimap() : root_{new NLeafT(nullptr, nullptr, nullptr)}, size_{0} {};

    b_plus_tree_multimap(const b_plus_tree_multimap& other) : size_{other.size_} {
        root_ = other.root_->is_leaf() ? (NodeT*)new NLeafT(*other.root_->as_leaf())
                                       : (NodeT*)new NInnerT(*other.root_->as_inner());
    }

    b_plus_tree_multimap(b_plus_tree_multimap&& other) noexcept
    : root_{other.root_}, size_{other.size_} {
        other.root_ = nullptr;
        other.size_ = 0;
    }

    b_plus_tree_multimap& operator=(const b_plus_tree_multimap& other) {
        assert(this != &other);
        delete root_;
        root_ = other.root_->is_leaf() ? (NodeT*)new NLeafT(*other.root_->as_leaf())
                                       : (NodeT*)new NInnerT(*other.root_->as_inner());
        size_ = other.size_;
        return *this;
    }

    b_plus_tree_multimap& operator=(b_plus_tree_multimap&& other) noexcept {
        delete root_;
        root_ = other.root_;
        other.root_ = nullptr;
        size_ = other.size_;
        other.size_ = 0;
        return *this;
    }

    ~b_plus_tree_multimap() noexcept {
        delete root_;
    }

    [[nodiscard]] auto find(const KeyT key) {
        auto leaf = lower_bound_leaf(key, root_);
        return leaf != nullptr ? leaf->find(key) : IterT{};
    }

    [[nodiscard]] auto find(const KeyT key) const {
        return const_cast<b_plus_tree_multimap&>(*this).find(key);
    }

    [[nodiscard]] size_t count(const KeyT key) const {
        return const_cast<b_plus_tree_multimap&>(*this).find(key) != end();
    }

    [[nodiscard]] auto lower_bound(const KeyT key) {
        auto leaf = lower_bound_leaf(key, root_);
        return leaf != nullptr ? leaf->template lower_bound_as_iter<IterT>(key) : IterT{};
    }

    [[nodiscard]] auto lower_bound(const KeyT key) const {
        return const_cast<b_plus_tree_multimap&>(*this).lower_bound(key);
    }

    [[nodiscard]] auto begin() noexcept {
        return IterT(root_);
    }

    [[nodiscard]] auto begin() const noexcept {
        return IterT(const_cast<NodeT*>(root_));
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

    template <typename... Args>
    auto emplace(KeyT key, Args&&... args) {
        auto leaf = lower_bound_or_last_leaf(key, root_);
        return leaf->try_emplace(key, root_, size_, std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto try_emplace(KeyT key, Args&&... args) {
        return emplace(key, std::forward<Args>(args)...);
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
    auto try_emplace(const IterT& hint, KeyT key, Args&&... args) {
        return emplace_hint(hint, key, std::forward<Args>(args)...);
    }

    size_t erase(const KeyT key) {
        auto begin = lower_bound(key);
        auto end = key == std::numeric_limits<KeyT>::max() ? IterT() : lower_bound(key + 1);
        if (begin == end) {
            return 0;
        }
        auto size_before = size_;
        erase(begin, end);
        return size_before - size_;
    }

    auto erase(const IterT& iterator) {
        assert(iterator != end());
        --size_;
        auto result = iterator.node_->erase_entry(iterator.iter_, root_);
        if (result.node_) {
            return IterT(result.node_->as_leaf(), result.iter_);
        }
        return IterT();
    }

    auto erase(const IterT& begin, const IterT& end) {
        assert(begin != this->end());
        NLeafT* current = begin.node_;
        auto current_begin = begin.iter_;
        size_t end_offset = 0;
        if (!end.is_end()) {
            if (begin.node_ == end.node_) {
                // No page merge, but end_offset depends on "begin" iterator
                end_offset = end.iter_ - begin.iter_;
            } else {
                // The end iterator may be invalidated by page merges!
                end_offset = end.iter_ - end.node_->data_.begin();
            }
        }
        size_t n_erased = 0;
        while (current != end.node_ && current->next_node_ != nullptr) {
            auto old_size = current->data_.size();
            KeyT max_key_old = current->data_.back().first;
            current->data_.erase(current_begin, current->data_.end());
            n_erased += (old_size - current->data_.size());
            auto result = current->check_merge(current->data_.end(), max_key_old, root_);
            current = result.node_->as_leaf();
            assert(current != nullptr);
            current_begin = result.iter_;
        }
        auto old_size = current->data_.size();
        KeyT max_key_old = current->data_.back().first;
        auto current_end = end.is_end() ? current->data_.end() : current_begin + end_offset;
        auto next_entry = current->data_.erase(current_begin, current_end);
        n_erased += (old_size - current->data_.size());
        auto result = current->check_merge(next_entry, max_key_old, root_);
        size_ -= n_erased;
        if (result.node_) {
            return IterT(result.node_->as_leaf(), result.iter_);
        }
        return IterT();
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
    using bpt_leaf_super = detail::bpt_node_data<KeyT, ValueT, NInnerT, NLeafT, true>;
    class bpt_node_leaf : public bpt_leaf_super {
      public:
        explicit bpt_node_leaf(NInnerT* parent, NLeafT* prev, NLeafT* next) noexcept
        : bpt_leaf_super(true, parent, prev, next) {}

        ~bpt_node_leaf() noexcept = default;

        [[nodiscard]] IterT find(KeyT key) noexcept {
            IterT iter_full = this->template lower_bound_as_iter<IterT>(key);
            if (!iter_full.is_end() && iter_full.iter_->first == key) {
                return iter_full;
            }
            return IterT();
        }

        template <typename... Args>
        auto try_emplace(KeyT key, NodeT*& root, size_t& entry_count, Args&&... args) {
            auto it = this->lower_bound(key);
            ++entry_count;
            auto full_it = this->template check_split_and_adjust_iterator<IterT>(it, key, root);
            auto it_result =
                full_it.node_->data_.emplace(full_it.iter_, key, std::forward<Args>(args)...);
            return IterT(full_it.node_, it_result);
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
                assert(count == 0 || e.first >= known_min);
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

#endif  // PHTREE_COMMON_B_PLUS_TREE_MULTIMAP_H
