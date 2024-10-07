/*
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

#ifndef PHTREE_COMMON_B_PLUS_TREE_BASE_H
#define PHTREE_COMMON_B_PLUS_TREE_BASE_H

#include "bpt_fixed_vector.h"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <tuple>
#include <vector>

namespace phtree::bptree::detail {

enum bpt_config_container {
    VAR,    // std::vector
    FIXED,  // bpt_vector
};

template <size_t MAX_, size_t MIN_, size_t INIT_, bpt_config_container MODE_ = VAR>
struct bpt_config {
    static constexpr size_t MAX = MAX_;
    static constexpr size_t MIN = MIN_;
    static constexpr size_t INIT = INIT_;
    static constexpr bpt_config_container MODE = MODE_;
};

template <typename KeyT, typename NInnerT, typename NLeafT>
class bpt_node_base {
  public:
    explicit bpt_node_base(bool is_leaf, NInnerT* parent) noexcept
    : is_leaf_{is_leaf}, parent_{parent} {}

    virtual ~bpt_node_base() noexcept = default;

    [[nodiscard]] constexpr bool is_leaf() const noexcept {
        return is_leaf_;
    }

    [[nodiscard]] constexpr NInnerT* as_inner() noexcept {
        assert(!is_leaf_);
        return static_cast<NInnerT*>(this);
    }

    [[nodiscard]] constexpr NLeafT* as_leaf() noexcept {
        assert(is_leaf_);
        return static_cast<NLeafT*>(this);
    }

    virtual void _check(size_t&, const NInnerT*, const NLeafT*&, KeyT&, KeyT) const = 0;

  private:
    const bool is_leaf_;

  public:
    NInnerT* parent_;
};

template <
    typename KeyT,
    typename ValueT,
    typename NInnerT,
    typename NLeafT,
    bool IsLeaf,
    typename CFG = bpt_config<16, 2, 2>>
class bpt_node_data : public bpt_node_base<KeyT, NInnerT, NLeafT> {
    // TODO This could be improved but requires a code change to move > 1 entry when merging.
    static_assert(CFG::MIN == 2 && "M_MIN != 2 is not supported");
    using ValueT2 = std::conditional_t<IsLeaf, ValueT, bpt_node_base<KeyT, NInnerT, NLeafT>*>;
    using EntryT = std::pair<KeyT, ValueT2>;
    using ContainerT = std::conditional_t<
        CFG::MODE == bpt_config_container::VAR,
        std::vector<EntryT>,
        bpt_vector<EntryT, CFG::MAX>>;
    using ThisT = std::conditional_t<IsLeaf, NLeafT, NInnerT>;

  public:
    using DataIteratorT = typename ContainerT::iterator;
    // MSVC++ requires this to be public, otherwise there it clashes with sub-classes' NodeT!?!?!
    using NodeT = bpt_node_base<KeyT, NInnerT, NLeafT>;
    explicit bpt_node_data(bool is_leaf, NInnerT* parent, ThisT* prev, ThisT* next) noexcept
    : bpt_node_base<KeyT, NInnerT, NLeafT>(is_leaf, parent)
    , data_{}
    , prev_node_{prev}
    , next_node_{next} {
        data_.reserve(CFG::INIT);
    }

    virtual ~bpt_node_data() noexcept = default;

    [[nodiscard]] auto lower_bound(KeyT key) noexcept {
        // If this doesn´t compile, check #include <algorithm> !!!
        return std::lower_bound(data_.begin(), data_.end(), key, [](EntryT& left, const KeyT key) {
            return left.first < key;
        });
    }

    template <typename IterT>
    [[nodiscard]] IterT lower_bound_as_iter(KeyT key) noexcept {
        auto it = lower_bound(key);
        return it == data_.end() ? IterT{} : IterT(this->as_leaf(), std::move(it));
    }

    [[nodiscard]] size_t size() const noexcept {
        return data_.size();
    }

    struct EraseResult {
        bpt_node_data* node_ = nullptr;
        DataIteratorT iter_;
    };

    auto erase_entry(DataIteratorT it_to_erase, NodeT*& root) {
        auto max_key = data_.back().first;
        auto it_after_erased = data_.erase(it_to_erase);
        return check_merge(it_after_erased, max_key, root);
    }

    auto check_merge(DataIteratorT iter_after_erased, KeyT max_key_old, NodeT*& root) {
        using ER = EraseResult;
        auto& parent_ = this->parent_;
        bool tail_entry_erased = iter_after_erased == data_.end();

        if (parent_ == nullptr) {
            if constexpr (std::is_same_v<ThisT, NInnerT>) {
                if (data_.size() < 2) {
                    auto remaining_node = data_.begin()->second;
                    data_.begin()->second = nullptr;
                    remaining_node->parent_ = nullptr;
                    root = remaining_node;
                    delete this;
                }
            }
            return tail_entry_erased ? ER{} : ER{this, iter_after_erased};
        }

        if (data_.empty()) {
            // Nothing to merge, just remove node. This should be rare, i.e. only happens when
            // a rare 1-entry node has its last entry removed.
            auto next_node = next_node_;  // create copy because (this) will be deleted
            remove_from_siblings();
            parent_->remove_node(max_key_old, this, root);
            return next_node == nullptr ? ER{} : ER{next_node, next_node->data_.begin()};
        }

        if (data_.size() < CFG::MIN) {
            // merge
            if (prev_node_ != nullptr && prev_node_->data_.size() < CFG::MAX) {
                remove_from_siblings();
                auto& prev_data = prev_node_->data_;
                if constexpr (std::is_same_v<ThisT, NLeafT>) {
                    prev_data.emplace_back(std::move(data_[0]));
                } else {
                    data_[0].second->parent_ = prev_node_;
                    prev_data.emplace_back(std::move(data_[0]));
                    data_[0].second = nullptr;
                }
                auto prev_node = prev_node_;  // create copy because (this) will be deleted
                auto next_node = next_node_;  // create copy because (this) will be deleted
                parent_->remove_node(max_key_old, this, root);
                if (prev_node->parent_ != nullptr) {
                    KeyT old1 = (prev_data.end() - 2)->first;
                    KeyT new1 = (prev_data.end() - 1)->first;
                    prev_node->parent_->update_key(old1, new1, prev_node);
                }
                if (!tail_entry_erased) {
                    return ER{prev_node, prev_data.end() - 1};
                }
                return next_node == nullptr ? ER{} : ER{next_node, next_node->data_.begin()};
            } else if (next_node_ != nullptr && next_node_->data_.size() < CFG::MAX) {
                remove_from_siblings();
                auto* next_node = next_node_;
                auto& next_data = next_node_->data_;
                if constexpr (std::is_same_v<ThisT, NLeafT>) {
                    next_data.emplace(next_data.begin(), std::move(data_[0]));
                } else {
                    data_[0].second->parent_ = next_node_;
                    next_data.emplace(next_data.begin(), std::move(data_[0]));
                    data_[0].second = nullptr;
                }
                parent_->remove_node(max_key_old, this, root);
                if (tail_entry_erased) {
                    return ER{next_node, next_data.begin() + 1};
                }
                return next_node == nullptr ? ER() : ER{next_node, next_data.begin()};
            }
            // This node is too small but there is nothing we can do.
        }
        if (tail_entry_erased) {
            parent_->update_key(max_key_old, data_.back().first, this);
            return next_node_ == nullptr ? ER() : ER{next_node_, next_node_->data_.begin()};
        }
        return ER{this, iter_after_erased};
    }

    /*
     * Check whether a split is required and, if so, perform it.
     */
    bool check_split(NodeT*& root) {
        if (data_.size() >= CFG::MAX) {
            if (!this->rebalance()) {
                this->split_node(root);
            }
            return true;
        }
        return false;
    }

    template <typename IterT>
    auto check_split_and_adjust_iterator(DataIteratorT it, KeyT key, NodeT*& root) {
        auto* dest = (ThisT*)this;
        bool is_split = this->check_split(root);
        if (is_split && key > this->data_.back().first) {
            dest = this->next_node_;
            it = dest->lower_bound(key);
        }

        if (dest->parent_ != nullptr && key > dest->data_.back().first) {
            dest->parent_->update_key(dest->data_.back().first, key, dest);
        }

        return IterT(dest, it);
    }

    void _check_data(const NInnerT* parent, KeyT known_max) const {
        (void)parent;
        (void)known_max;
        // assert(parent_ == nullptr || data_.size() >= CFG::MIN);
        assert(this->parent_ == parent);
        if (this->data_.empty()) {
            assert(parent == nullptr);
            return;
        }
        assert(this->parent_ == nullptr || known_max == this->data_.back().first);
    }

  private:
    void split_node(NodeT*& root) {
        auto max_key = data_.back().first;
        if (this->parent_ == nullptr) {
            auto* new_parent = new NInnerT(nullptr, nullptr, nullptr);
            new_parent->emplace_back(max_key, this);
            root = new_parent;
            this->parent_ = new_parent;
        }

        // create new node
        auto* node2 = new ThisT(this->parent_, static_cast<ThisT*>(this), next_node_);
        if (next_node_ != nullptr) {
            next_node_->prev_node_ = node2;
        }
        next_node_ = node2;

        // populate new node
        // TODO Optimize populating new node: move 1st part, insert new value, move 2nd part...?
        auto split_pos = CFG::MAX >> 1;
        node2->data_.insert(
            node2->data_.end(),
            std::make_move_iterator(data_.begin() + split_pos),
            std::make_move_iterator(data_.end()));
        data_.erase(data_.begin() + split_pos, data_.end());

        if constexpr (std::is_same_v<ThisT, NInnerT>) {
            for (auto& e : node2->data_) {
                e.second->parent_ = node2;
            }
        }

        // Add node to parent
        auto split_key = data_.back().first;
        this->parent_->update_key_and_add_node(max_key, split_key, this, node2, root);
    }

    bool rebalance() {
        // We rebalance to "next" if it has at least 50% free space.
        // Rebalancing to "prev" is difficult because update_key_and_add_node() relies on "next".
        size_t threshold = CFG::MAX >> 1;  // 50%
        size_t move_amount = CFG::MAX >> 2;
        auto& data = this->data_;
        if (this->next_node_ != nullptr && CFG::MAX - next_node_->data_.size() > threshold) {
            auto& next_data = this->next_node_->data_;
            auto old_key = data.back().first;
            auto start = data.end() - move_amount;
            auto end = data.end();
            next_data.insert(
                next_data.begin(), std::make_move_iterator(start), std::make_move_iterator(end));
            data.erase(start, end);
            if constexpr (std::is_same_v<ThisT, NInnerT>) {
                auto it = next_data.begin();
                for (size_t i = 0; i < move_amount; ++i) {
                    it->second->parent_ = this->next_node_;
                    ++it;
                }
            }
            this->parent_->update_key(old_key, data.back().first, this);
            return true;
        }
        return false;
    }

    void remove_from_siblings() {
        if (next_node_ != nullptr) {
            next_node_->prev_node_ = prev_node_;
        }
        if (prev_node_ != nullptr) {
            prev_node_->next_node_ = next_node_;
        }
    }

  public:
    ContainerT data_;
    ThisT* prev_node_;
    ThisT* next_node_;
};

// This type is replaced with the proper type inside bpt_node_data
using bpt_dummy = void;

template <typename KeyT, typename NLeafT, typename CFG = bpt_config<16, 2, 2>>
class bpt_node_inner
: public bpt_node_data<KeyT, bpt_dummy, bpt_node_inner<KeyT, NLeafT, CFG>, NLeafT, false, CFG> {
    using NInnerT = bpt_node_inner<KeyT, NLeafT, CFG>;
    using NodePtrT = bpt_node_base<KeyT, NInnerT, NLeafT>*;

  public:
    explicit bpt_node_inner(NInnerT* parent, NInnerT* prev, NInnerT* next) noexcept
    : bpt_node_data<KeyT, bpt_dummy, NInnerT, NLeafT, false, CFG>(false, parent, prev, next) {}

    ~bpt_node_inner() noexcept {
        for (auto& e : this->data_) {
            if (e.second != nullptr) {
                delete e.second;
            }
        }
    }

    [[nodiscard]] auto lower_bound_node(KeyT key, const NodePtrT node) noexcept {
        auto it = this->lower_bound(key);
        while (it != this->data_.end() && it->first == key) {
            if (it->second == node) {
                return it;
            }
            ++it;
        }
        return this->data_.end();
    }

    void emplace_back(KeyT key, NodePtrT node) {
        this->data_.emplace_back(key, node);
    }

    void _check(
        size_t& count,
        const NInnerT* parent,
        const NLeafT*& prev_leaf,
        KeyT& known_min,
        KeyT known_max) const {
        this->_check_data(parent, known_max);

        assert(this->parent_ == nullptr || known_max == this->data_.back().first);
        auto prev_key = this->data_[0].first;
        size_t n = 0;
        for (auto& e : this->data_) {
            assert(n == 0 || e.first >= prev_key);
            e.second->_check(count, this, prev_leaf, known_min, e.first);
            assert(this->parent_ == nullptr || e.first <= known_max);
            prev_key = e.first;
            ++n;
        }
    }

    void update_key(KeyT old_key, KeyT new_key, NodePtrT node) {
        if (old_key == new_key) {
            return;  // This can happen due to multiple entries with same key.
        }
        auto it = this->lower_bound_node(old_key, node);
        assert(it != this->data_.end() && it->first == old_key);
        it->first = new_key;
        if (this->parent_ != nullptr && ++it == this->data_.end()) {
            this->parent_->update_key(old_key, new_key, this);
        }
    }

    /*
     * This method does two things:
     * - It changes the key of the node (node 1) at 'key1_old' to 'key1_new'.
     * - It inserts a new node (node 2) after 'new_key1' with key='key1_old' (it's max key)
     * Invariants:
     * - Node1: key1_old >= key1_new
     */
    void update_key_and_add_node(
        KeyT key1_old, KeyT key1_new, NodePtrT child1, NodePtrT child2, NodePtrT& root) {
        bool has_split = this->check_split(root);

        // splits are always "forward", i.e. creating a "next" node. How about rebalance()?
        auto* dest = this;
        if (has_split && key1_old > this->data_.back().first) {
            dest = this->next_node_;
        }

        // update child1
        auto it = dest->lower_bound_node(key1_old, child1);
        assert(key1_old >= key1_new && it != dest->data_.end());
        it->first = key1_new;

        if (dest == this && this->next_node_ != nullptr) {
            assert(this->next_node_->data_.front().first >= key1_new);
        }
        ++it;
        // key_1_old is the max_key of child2
        dest->data_.emplace(it, key1_old, child2);
        child2->parent_ = dest;
    }

    void remove_node(KeyT key_remove, NodePtrT node, NodePtrT& root) {
        auto it_to_erase = this->lower_bound(key_remove);
        while (it_to_erase != this->data_.end() && it_to_erase->first == key_remove) {
            if (it_to_erase->second == node) {
                delete it_to_erase->second;
                this->erase_entry(it_to_erase, root);
                return;
            }
            ++it_to_erase;
        }
        assert(false && "Node not found!");
    }
};

template <typename NLeafT, typename NodeT, typename F1>
class bpt_iterator_base {
    using IterT = bpt_iterator_base<NLeafT, NodeT, F1>;

    template <typename A, typename B, typename C, typename D, bool E, typename F>
    friend class bpt_node_data;
    friend F1;
    friend NLeafT;

  protected:
    using LeafIteratorT = typename NLeafT::DataIteratorT;

  public:
    // Arbitrary position iterator
    explicit bpt_iterator_base(NLeafT* node, LeafIteratorT it) noexcept : node_{node}, iter_{it} {
        assert(
            (node == nullptr || node->is_leaf()) &&
            "for consistency, insist that we iterate leaves only");
    }

    // begin() iterator
    explicit bpt_iterator_base(NodeT* node) noexcept {
        assert(node->parent_ == nullptr && "must start with root node");
        // move iterator to first value
        while (!node->is_leaf()) {
            node = node->as_inner()->data_[0].second;
        }
        node_ = node->as_leaf();

        if (node_->size() == 0) {
            node_ = nullptr;
            iter_ = {};
            return;
        }
        iter_ = node_->data_.begin();
    }

    // end() iterator
    bpt_iterator_base() noexcept : node_{nullptr}, iter_{} {}

    auto operator++() noexcept {
        assert(!is_end());
        ++iter_;
        if (iter_ == node_->data_.end()) {
            // this may be a nullptr -> end of data
            node_ = node_->next_node_;
            iter_ = node_ != nullptr ? node_->data_.begin() : LeafIteratorT{};
        }
        return *this;
    }

    auto operator++(int) const noexcept {
        IterT iterator(*this);
        ++(*this);
        return iterator;
    }

    friend bool operator==(const IterT& left, const IterT& right) noexcept {
        return left.node_ == right.node_ && left.iter_ == right.iter_;
    }

    friend bool operator!=(const IterT& left, const IterT& right) noexcept {
        return left.node_ != right.node_ || left.iter_ != right.iter_;
    }

  protected:
    LeafIteratorT& iter() const noexcept {
        return const_cast<LeafIteratorT&>(iter_);
    }

  private:
    [[nodiscard]] bool is_end() const noexcept {
        return node_ == nullptr;
    }

    NLeafT* node_;
    LeafIteratorT iter_;
};

template <typename KeyT, typename NodeT>
[[nodiscard]] static auto lower_bound_leaf(KeyT key, NodeT* node) noexcept {
    using LeafT = decltype(node->as_leaf());
    while (node != nullptr && !node->is_leaf()) {
        auto it = node->as_inner()->lower_bound(key);
        node = it != node->as_inner()->data_.end() ? it->second : nullptr;
    }
    return static_cast<LeafT>(node);  // `node` may be nullptr
}

template <typename KeyT, typename NodeT>
[[nodiscard]] static auto lower_bound_or_last_leaf(KeyT key, NodeT* node) noexcept {
    while (!node->is_leaf()) {
        auto it = node->as_inner()->lower_bound(key);
        if (it == node->as_inner()->data_.end()) {
            node = node->as_inner()->data_.back().second;
        } else {
            node = it->second;
        }
    }
    return node->as_leaf();
}

}  // namespace phtree::bptree::detail

#endif  // PHTREE_COMMON_B_PLUS_TREE_BASE_H
