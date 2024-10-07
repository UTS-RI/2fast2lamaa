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

#ifndef PHTREE_COMMON_FLAT_SPARSE_MAP_H
#define PHTREE_COMMON_FLAT_SPARSE_MAP_H

#include "bits.h"
#include <cassert>
#include <tuple>
#include <vector>

/*
 * PLEASE do not include this file directly, it is included via common.h.
 *
 * This file contains the sparse_map implementation, which is used in medium-dimensional nodes in
 * the PH-Tree.
 */
namespace improbable::phtree::detail {

/*
 * The sparse_map is a flat map implementation that uses an array of *at* *most* SIZE=2^DIM.
 * The array contains a list sorted by key.
 *
 * It has O(log n) lookup and O(n) insertion/removal time complexity, space complexity is O(n).
 */
template <typename KeyT, typename ValueT>
class sparse_map {
    using Entry = std::pair<KeyT, ValueT>;
    using iterator = typename std::vector<Entry>::iterator;

  public:
    explicit sparse_map() : data_{} {
        data_.reserve(4);
    }

    [[nodiscard]] auto find(KeyT key) {
        auto it = lower_bound(key);
        if (it != data_.end() && it->first == key) {
            return it;
        }
        return data_.end();
    }

    [[nodiscard]] auto find(KeyT key) const {
        auto it = lower_bound(key);
        if (it != data_.end() && it->first == key) {
            return it;
        }
        return data_.end();
    }

    [[nodiscard]] auto lower_bound(KeyT key) {
        return std::lower_bound(data_.begin(), data_.end(), key, [](Entry& left, const KeyT key) {
            return left.first < key;
        });
    }

    [[nodiscard]] auto lower_bound(KeyT key) const {
        return std::lower_bound(
            data_.cbegin(), data_.cend(), key, [](const Entry& left, const KeyT key) {
                return left.first < key;
            });
    }

    [[nodiscard]] auto begin() noexcept {
        return data_.begin();
    }

    [[nodiscard]] auto begin() const noexcept {
        return cbegin();
    }

    [[nodiscard]] auto cbegin() const noexcept {
        return data_.cbegin();
    }

    [[nodiscard]] auto end() noexcept {
        return data_.end();
    }

    [[nodiscard]] auto end() const noexcept {
        return data_.end();
    }

    [[nodiscard]] auto cend() const noexcept {
        return data_.cend();
    }

    template <typename... Args>
    auto emplace(KeyT key, Args&&... args) {
        auto iter = lower_bound(key);
        return try_emplace_base(iter, key, std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto try_emplace(KeyT key, Args&&... args) {
        auto iter = lower_bound(key);
        return try_emplace_base(iter, key, std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto try_emplace(iterator iter, KeyT key, Args&&... args) {
        return try_emplace_base(iter, key, std::forward<Args>(args)...).first;
    }

    void erase(KeyT key) {
        auto it = lower_bound(key);
        if (it != end() && it->first == key) {
            data_.erase(it);
        }
    }

    void erase(const iterator& iter) {
        data_.erase(iter);
    }

    [[nodiscard]] size_t size() const {
        return data_.size();
    }

  private:
    template <typename... Args>
    auto try_emplace_base(const iterator& it, KeyT key, Args&&... args) {
        if (it != end() && it->first == key) {
            return std::make_pair(it, false);
        } else {
            auto x = data_.emplace(
                it,
                std::piecewise_construct,
                std::forward_as_tuple(key),
                std::forward_as_tuple(std::forward<Args>(args)...));
            return std::make_pair(x, true);
        }
    }

    std::vector<Entry> data_;
};

}  // namespace improbable::phtree

#endif  // PHTREE_COMMON_FLAT_SPARSE_MAP_H
