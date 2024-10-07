/*
 * Copyright 2020 Improbable Worlds Limited
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

#ifndef PHTREE_COMMON_FLAT_ARRAY_MAP_H
#define PHTREE_COMMON_FLAT_ARRAY_MAP_H

#include "bits.h"
#include <bitset>
#include <cassert>
#include <tuple>

/*
 * PLEASE do not include this file directly, it is included via common.h.
 *
 * This file contains the array_map implementation, which is used in low-dimensional nodes in the
 * PH-Tree.
 */
namespace improbable::phtree::detail {

template <typename Key, typename Value, Key SIZE>
class flat_array_map;

namespace {

template <typename Key, typename Value>
using flat_map_pair = std::pair<Key, Value>;

template <typename Key, typename Value, Key SIZE>
class flat_map_iterator {
    friend flat_array_map<Key, Value, SIZE>;

  public:
    flat_map_iterator() : first{0}, map_{nullptr} {};

    explicit flat_map_iterator(Key index, const flat_array_map<Key, Value, SIZE>* map)
    : first{index}, map_{map} {
        assert(index <= SIZE);
    }

    auto& operator*() const {
        assert(first < SIZE && map_->occupied(first));
        return const_cast<flat_map_pair<Key, Value>&>(map_->data(first));
    }

    auto* operator->() const {
        assert(first < SIZE && map_->occupied(first));
        return const_cast<flat_map_pair<Key, Value>*>(&map_->data(first));
    }

    auto& operator++() noexcept {
        first = (first + 1) >= SIZE ? SIZE : map_->lower_bound_index(first + 1);
        return *this;
    }

    auto operator++(int) noexcept {
        flat_map_iterator it(first, map_);
        ++(*this);
        return it;
    }

    friend bool operator==(const flat_map_iterator& left, const flat_map_iterator& right) {
        return left.first == right.first;
    }

    friend bool operator!=(const flat_map_iterator& left, const flat_map_iterator& right) {
        return left.first != right.first;
    }

  private:
    Key first;
    const flat_array_map<Key, Value, SIZE>* map_;
};
}  // namespace

/*
 * The array_map is a flat map implementation that uses an array of SIZE=2^DIM. The key is
 * effectively the position in the array.
 *
 * It has O(1) insertion/removal time complexity, but O(2^DIM) space complexity, so it is best used
 * when DIM is low and/or the map is known to have a high fill ratio.
 */
template <typename Key, typename Value, Key SIZE>
class flat_array_map {
    static_assert(std::is_integral<Key>() && "Key type must be integer");
    static_assert(std::is_unsigned<Key>() && "Key type must unsigned");
    using map_pair = detail::flat_map_pair<Key, Value>;
    using iterator = detail::flat_map_iterator<Key, Value, SIZE>;
    friend iterator;

  public:
    [[nodiscard]] auto find(Key index) noexcept {
        return iterator{occupied(index) ? index : SIZE, this};
    }

    [[nodiscard]] auto lower_bound(Key index) const noexcept {
        return iterator{lower_bound_index(index), this};
    }

    [[nodiscard]] auto begin() const noexcept {
        return iterator{lower_bound_index(0), this};
    }

    [[nodiscard]] auto cbegin() const noexcept {
        return iterator{lower_bound_index(0), this};
    }

    [[nodiscard]] auto end() const noexcept {
        return iterator{SIZE, this};
    }

    [[nodiscard]] auto cend() const noexcept {
        return iterator{SIZE, this};
    }

    ~flat_array_map() noexcept {
        if (occupancy != 0) {
            for (Key i = 0; i < SIZE; ++i) {
                if (occupied(i)) {
                    data(i).~pair();
                }
            }
        }
    }

    [[nodiscard]] size_t size() const noexcept {
        constexpr size_t BITS =
            std::numeric_limits<Key>::digits + std::numeric_limits<Key>::is_signed;
        return std::bitset<BITS>(occupancy).count();
    }

    template <typename... Args>
    std::pair<map_pair*, bool> try_emplace(Key index, Args&&... args) {
        if (!occupied(index)) {
            new (reinterpret_cast<void*>(&data_[index])) map_pair(
                std::piecewise_construct,
                std::forward_as_tuple(index),
                std::forward_as_tuple(std::forward<Args>(args)...));
            occupy(index);
            return {&data(index), true};
        }
        return {&data(index), false};
    }

    bool erase(Key index) noexcept {
        if (occupied(index)) {
            data(index).~pair();
            unoccupy(index);
            return true;
        }
        return false;
    }

    bool erase(const iterator& iterator) noexcept {
        return erase(iterator.first);
    }

  private:
    /*
     * This returns the element at the given index, which is _not_ the n'th element (for n = index).
     */
    map_pair& data(Key index) noexcept {
        assert(occupied(index));
        return *std::launder(reinterpret_cast<map_pair*>(&data_[index]));
    }

    const map_pair& data(Key index) const noexcept {
        assert(occupied(index));
        return *std::launder(reinterpret_cast<const map_pair*>(&data_[index]));
    }

    [[nodiscard]] Key lower_bound_index(Key index) const noexcept {
        assert(index < SIZE);
        Key num_zeros = CountTrailingZeros64(occupancy >> index);
        // num_zeros may be equal to SIZE if no bits remain
        return std::min(SIZE, index + num_zeros);
    }

    void occupy(Key index) noexcept {
        assert(index < SIZE);
        assert(!occupied(index));
        // flip the bit
        occupancy ^= (Key{1} << index);
    }

    void unoccupy(Key index) noexcept {
        assert(index < SIZE);
        assert(occupied(index));
        // flip the bit
        occupancy ^= (Key{1} << index);
    }

    [[nodiscard]] bool occupied(Key index) const noexcept {
        return (occupancy >> index) & Key{1};
    }

    Key occupancy = 0;
    // We use an untyped array to avoid implicit calls to constructors and destructors of entries.
    std::aligned_storage_t<sizeof(map_pair), alignof(map_pair)> data_[SIZE];
};

/*
 * array_map is a wrapper around flat_array_map. It introduces one layer of indirection.
 * This is useful to decouple instantiation of a node from instantiation of it's descendants
 * (the flat_array_map directly instantiates an array of descendants).
 */
template <typename Key, typename Value, Key SIZE>
class array_map {
    static_assert(SIZE <= 64);  // or else we need to adapt 'occupancy'
    static_assert(SIZE > 0);
    using iterator = improbable::phtree::detail::flat_map_iterator<Key, Value, SIZE>;

  public:
    array_map() {
        data_ = new flat_array_map<Key, Value, SIZE>();
    }

    array_map(const array_map& other) = delete;
    array_map& operator=(const array_map& other) = delete;

    array_map(array_map&& other) noexcept : data_{other.data_} {
        other.data_ = nullptr;
    }

    array_map& operator=(array_map&& other) noexcept {
        data_ = other.data_;
        other.data_ = nullptr;
        return *this;
    }

    ~array_map() {
        delete data_;
    }

    [[nodiscard]] auto find(Key index) noexcept {
        return data_->find(index);
    }

    [[nodiscard]] auto find(Key key) const noexcept {
        return const_cast<array_map&>(*this).find(key);
    }

    [[nodiscard]] auto lower_bound(Key index) const {
        return data_->lower_bound(index);
    }

    [[nodiscard]] auto begin() const noexcept {
        return data_->begin();
    }

    [[nodiscard]] iterator cbegin() const noexcept {
        return data_->cbegin();
    }

    [[nodiscard]] auto end() const noexcept {
        return data_->end();
    }

    [[nodiscard]] auto cend() const noexcept {
        return data_->cend();
    }

    template <typename... Args>
    auto emplace(Args&&... args) {
        return data_->try_emplace(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto try_emplace(Key index, Args&&... args) {
        return data_->try_emplace(index, std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto try_emplace(const iterator&, Key index, Args&&... args) {
        // We ignore the iterator, this is an array based collection, so access is ~O(1).
        return data_->try_emplace(index, std::forward<Args>(args)...).first;
    }

    bool erase(Key index) {
        return data_->erase(index);
    }

    bool erase(const iterator& iterator) {
        return data_->erase(iterator);
    }

    [[nodiscard]] size_t size() const {
        return data_->size();
    }

  private:
    flat_array_map<Key, Value, SIZE>* data_;
};

}  // namespace improbable::phtree::detail

#endif  // PHTREE_COMMON_FLAT_ARRAY_MAP_H
