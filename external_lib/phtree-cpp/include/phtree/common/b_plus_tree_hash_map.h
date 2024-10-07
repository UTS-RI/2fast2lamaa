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

#ifndef PHTREE_COMMON_B_PLUS_TREE_HASH_MAP_H
#define PHTREE_COMMON_B_PLUS_TREE_HASH_MAP_H

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
 * The b_plus_tree_hash_map is a B+tree implementation that uses a hierarchy of horizontally
 * connected nodes for fast traversal through all entries.
 *
 * Behavior
 * ========
 * This is a hash set/map. It behaves just like std::unordered_set / std::unordered_map, minus
 * some API functions.
 * The set/map is ordered by their hash.  Entries with identical hash have no specific ordering
 * but the order is stable with respect to insertion/removal of other entries.
 *
 *
 * Rationale
 * =========
 * This implementations is optimized for small entry count (for the multi-map PH-tree we
 * expect small numbers of entries that actually have identical positions), however it should
 * scale well with large entry counts (it is a tree, so there is no need for rehashing).
 * Benchmarks show 10%-20% performance improvements for relocate() when using this custom set/map.
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
template <typename T, typename HashT = std::hash<T>, typename PredT = std::equal_to<T>>
class b_plus_tree_hash_set {
    using hash_t = std::uint32_t;

    class bpt_node_leaf;
    class bpt_iterator;
    using IterT = bpt_iterator;
    using NLeafT = bpt_node_leaf;
    using NInnerT = detail::bpt_node_inner<hash_t, NLeafT>;
    using NodeT = detail::bpt_node_base<hash_t, NInnerT, bpt_node_leaf>;
    using TreeT = b_plus_tree_hash_set<T, HashT, PredT>;

  public:
    using value_compare = PredT;
    explicit b_plus_tree_hash_set() : root_{new NLeafT(nullptr, nullptr, nullptr)}, size_{0} {};

    b_plus_tree_hash_set(const b_plus_tree_hash_set& other) : size_{other.size_} {
        root_ = other.root_->is_leaf() ? (NodeT*)new NLeafT(*other.root_->as_leaf())
                                       : (NodeT*)new NInnerT(*other.root_->as_inner());
    }

    b_plus_tree_hash_set(b_plus_tree_hash_set&& other) noexcept
    : root_{other.root_}, size_{other.size_} {
        other.root_ = nullptr;
        other.size_ = 0;
    }

    b_plus_tree_hash_set& operator=(const b_plus_tree_hash_set& other) {
        assert(this != &other);
        delete root_;
        root_ = other.root_->is_leaf() ? (NodeT*)new NLeafT(*other.root_->as_leaf())
                                       : (NodeT*)new NInnerT(*other.root_->as_inner());
        size_ = other.size_;
        return *this;
    }

    b_plus_tree_hash_set& operator=(b_plus_tree_hash_set&& other) noexcept {
        delete root_;
        root_ = other.root_;
        other.root_ = nullptr;
        size_ = other.size_;
        other.size_ = 0;
        return *this;
    }

    ~b_plus_tree_hash_set() noexcept {
        delete root_;
    }

    [[nodiscard]] auto find(const T& value) {
        auto hash = (hash_t)HashT{}(value);
        auto leaf = lower_bound_leaf(hash, root_);
        return leaf != nullptr ? leaf->find(hash, value) : IterT{};
    }

    [[nodiscard]] auto find(const T& value) const {
        return const_cast<b_plus_tree_hash_set&>(*this).find(value);
    }

    [[nodiscard]] auto lower_bound(const T& value) {
        auto hash = (hash_t)HashT{}(value);
        auto leaf = lower_bound_leaf(hash, root_);
        return leaf != nullptr ? leaf->lower_bound_value(hash, value) : IterT{};
    }

    [[nodiscard]] auto lower_bound(const T& value) const {
        return const_cast<b_plus_tree_hash_set&>(*this).lower_bound(value);
    }

    [[nodiscard]] size_t count(const T& value) const {
        return const_cast<b_plus_tree_hash_set&>(*this).find(value) != end();
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

    template <typename... Args>
    auto emplace(Args&&... args) {
        T t(std::forward<Args>(args)...);
        auto hash = (hash_t)HashT{}(t);
        auto leaf = lower_bound_or_last_leaf(hash, root_);
        return leaf->try_emplace(hash, root_, size_, std::move(t));
    }

    template <typename... Args>
    auto emplace_hint(const IterT& hint, Args&&... args) {
        if (empty() || hint.is_end()) {
            return emplace(std::forward<Args>(args)...).first;
        }

        T t(std::forward<Args>(args)...);
        auto hash = (hash_t)HashT{}(t);
        auto node = hint.node_->as_leaf();

        // The following may drop a valid hint but is easy to check.
        if (node->data_.begin()->first > hash || (node->data_.end() - 1)->first < hash) {
            return emplace(std::move(t)).first;
        }

        return node->try_emplace(hash, root_, size_, std::move(t)).first;
    }

    size_t erase(const T& value) {
        auto hash = (hash_t)HashT{}(value);
        auto leaf = lower_bound_leaf(hash, root_);
        if (leaf == nullptr) {
            return 0;
        }

        auto iter = leaf->lower_bound_value(hash, value);
        if (!iter.is_end() && PredT{}(*iter, value)) {
            iter.node_->erase_entry(iter.iter_, root_);
            --size_;
            return 1;
        }
        return 0;
    }

    auto erase(const IterT& iterator) {
        assert(iterator != end());
        --size_;
        auto result = iterator.node_->erase_entry(iterator.iter_, root_);
        if (result.node_) {
            return IterT(static_cast<NLeafT*>(result.node_), result.iter_);
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
        hash_t known_min = std::numeric_limits<hash_t>::max();
        root_->_check(count, nullptr, prev_leaf, known_min, 0);
        assert(count == size());
    }

  private:
    using bpt_leaf_super = detail::bpt_node_data<hash_t, T, NInnerT, NLeafT, true>;
    class bpt_node_leaf : public bpt_leaf_super {
      public:
        explicit bpt_node_leaf(NInnerT* parent, NLeafT* prev, NLeafT* next) noexcept
        : bpt_leaf_super(true, parent, prev, next) {}

        ~bpt_node_leaf() noexcept = default;

        [[nodiscard]] IterT find(hash_t hash, const T& value) noexcept {
            PredT equals{};
            IterT iter_full = this->template lower_bound_as_iter<IterT>(hash);
            while (!iter_full.is_end() && iter_full.hash() == hash) {
                if (equals(*iter_full, value)) {
                    return iter_full;
                }
                ++iter_full;
            }
            return IterT();
        }

        [[nodiscard]] auto lower_bound_value(hash_t hash, const T& value) noexcept {
            PredT equals{};
            IterT iter_full = this->template lower_bound_as_iter<IterT>(hash);
            while (!iter_full.is_end() && iter_full.hash() == hash) {
                if (equals(*iter_full, value)) {
                    break;
                }
                ++iter_full;
            }
            return iter_full;
        }

        auto try_emplace(hash_t hash, NodeT*& root, size_t& entry_count, T&& t) {
            auto it = this->lower_bound(hash);
            if (it != this->data_.end() && it->first == hash) {
                // Hash collision !
                PredT equals{};
                IterT full_iter(this, it);
                while (!full_iter.is_end() && full_iter.hash() == hash) {
                    if (equals(*full_iter, t)) {
                        return std::make_pair(full_iter, false);
                    }
                    ++full_iter;
                }
            }
            ++entry_count;

            auto full_it = this->template check_split_and_adjust_iterator<IterT>(it, hash, root);
            auto it_result = full_it.node_->data_.emplace(full_it.iter_, hash, std::move(t));
            return std::make_pair(IterT(full_it.node_, it_result), true);
        }

        void _check(
            size_t& count,
            const NInnerT* parent,
            const NLeafT*& prev_leaf,
            hash_t& known_min,
            hash_t known_max) const {
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
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using pointer = T*;
        using reference = T&;

        // Arbitrary position iterator
        explicit bpt_iterator(NLeafT* node, typename SuperT::LeafIteratorT it) noexcept
        : SuperT(node, it) {}

        // begin() iterator
        explicit bpt_iterator(NodeT* node) noexcept : SuperT(node) {}

        // end() iterator
        bpt_iterator() noexcept : SuperT() {}

        auto& operator*() const noexcept {
            return const_cast<T&>(this->iter()->second);
        }

        auto* operator->() const noexcept {
            return const_cast<T*>(&this->iter()->second);
        }

        [[nodiscard]] auto hash() const noexcept {
            return this->iter()->first;
        }
    };

  private:
    NodeT* root_;
    size_t size_;
};

template <
    typename KeyT,
    typename ValueT,
    typename HashT = std::hash<KeyT>,
    typename PredT = std::equal_to<KeyT>>
class b_plus_tree_hash_map {
    class iterator;
    using IterT = iterator;
    using EntryT = std::pair<KeyT, ValueT>;

  public:
    using value_compare = PredT;
    b_plus_tree_hash_map() : map_{} {};

    b_plus_tree_hash_map(const b_plus_tree_hash_map&) = delete;
    b_plus_tree_hash_map(b_plus_tree_hash_map&&) noexcept = default;
    b_plus_tree_hash_map& operator=(const b_plus_tree_hash_map&) = delete;
    b_plus_tree_hash_map& operator=(b_plus_tree_hash_map&&) noexcept = default;
    ~b_plus_tree_hash_map() = default;

    auto begin() const {
        return IterT(map_.begin());
    }

    auto end() const {
        return IterT(map_.end());
    }

    auto find(const KeyT& key) const {
        return IterT(map_.find(EntryT{key, {}}));
    }

    [[nodiscard]] auto lower_bound(const KeyT& key) const {
        return IterT(map_.lower_bound(EntryT{key, {}}));
    }

    auto count(const KeyT& key) const {
        return map_.count(EntryT{key, {}});
    }

    template <typename... Args>
    auto emplace(Args&&... args) {
        return try_emplace(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto emplace_hint(const IterT& hint, Args&&... args) {
        return try_emplace(hint, std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto try_emplace(const KeyT& key, Args&&... args) {
        auto result = map_.emplace(key, std::forward<Args>(args)...);
        return std::make_pair(iterator(result.first), result.second);
    }

    template <typename... Args>
    auto try_emplace(const IterT& hint, const KeyT& key, Args&&... args) {
        auto result = map_.emplace_hint(hint.map_iter_, key, std::forward<Args>(args)...);
        return IterT(result);
    }

    auto erase(const KeyT& key) {
        return map_.erase({key, {}});
    }

    auto erase(const IterT& iterator) {
        return IterT(map_.erase(iterator.map_iter_));
    }

    auto size() const {
        return map_.size();
    }

    auto empty() const {
        return map_.empty();
    }

    void _check() const {
        map_._check();
    }

  private:
    struct EntryHashT {
        size_t operator()(const EntryT& x) const {
            return HashT{}(x.first);
        }
    };

    struct EntryEqualsT {
        bool operator()(const EntryT& x, const EntryT& y) const {
            return PredT{}(x.first, y.first);
        }
    };

    class iterator {
        using T = EntryT;
        using MapIterType =
            decltype(std::declval<b_plus_tree_hash_set<EntryT, EntryHashT, EntryEqualsT>>()
                         .begin());
        friend b_plus_tree_hash_map<KeyT, ValueT, HashT, PredT>;

      public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using pointer = T*;
        using reference = T&;

        explicit iterator(MapIterType map_iter) noexcept : map_iter_{map_iter} {}

        // end() iterator
        iterator() noexcept : map_iter_{} {}

        auto& operator*() const noexcept {
            return *map_iter_;
        }

        auto* operator->() const noexcept {
            return &*map_iter_;
        }

        auto& operator++() noexcept {
            ++map_iter_;
            return *this;
        }

        auto operator++(int) noexcept {
            IterT iterator(*this);
            ++(*this);
            return iterator;
        }

        friend bool operator==(const IterT& left, const IterT& right) noexcept {
            return left.map_iter_ == right.map_iter_;
        }

        friend bool operator!=(const IterT& left, const IterT& right) noexcept {
            return left.map_iter_ != right.map_iter_;
        }

      private:
        MapIterType map_iter_;
    };

    b_plus_tree_hash_set<EntryT, EntryHashT, EntryEqualsT> map_;
};

}  // namespace phtree::bptree

#endif  // PHTREE_COMMON_B_PLUS_TREE_HASH_MAP_H
