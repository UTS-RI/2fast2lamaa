/*
 * Copyright 2023 Tilmann Zäschke
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

#ifndef PHTREE_COMMON_BPT_PRIORITY_QUEUE_H
#define PHTREE_COMMON_BPT_PRIORITY_QUEUE_H

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <deque>
#include <vector>

namespace phtree::bptree::detail {

/**
 * A priority queue based on a sorted vector.
 */
template <typename V, typename Compare = std::less<>>
class priority_queue {
  public:
    // Member types
    using value_type = V;
    // using allocator_type = Allocator
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using reference = value_type&;
    using const_reference = const value_type&;

  public:
    explicit priority_queue() noexcept : data_{}, comp_{} {}

    explicit priority_queue(size_t initial_size) noexcept : data_{}, comp_{} {
        data_.reserve(initial_size);
    }

    priority_queue(const priority_queue& rhs) noexcept : data_(rhs.data_), comp_{} {}

    priority_queue(priority_queue&& rhs) noexcept : data_{std::move(rhs.data_)}, comp_{} {}

    // TODO use default functions?
    priority_queue& operator=(const priority_queue& rhs) noexcept {
        data_ = rhs.data_;
        comp_ = rhs.comp_;
        return *this;
    }

    priority_queue& operator=(priority_queue&& rhs) noexcept {
        data_ = std::move(rhs.data_);
        comp_ = std::move(rhs.comp_);
        return *this;
    }

    ~priority_queue() noexcept = default;

    const V& top() const {
        assert(!data_.empty());
        return data_.back();
    }

    // TODO rename bottom()
    const V& top_max() const {
        assert(!data_.empty());
        return data_.front();
    }

    V& top_max() {
        assert(!data_.empty());
        return data_.front();
    }

    void pop() {
        assert(!data_.empty());
        data_.pop_back();
    }

    void pop_max() {
        assert(!data_.empty());
        data_.erase(data_.begin());
        // data_.pop_front();
    }

    V& operator[](size_t index) noexcept {
        assert(index < data_.size());
        return data_[index];
    }

    const V& operator[](size_t index) const noexcept {
        assert(index < data_.size());
        return data_[index];
    }

    template <typename... Args>
    void emplace(Args&&... args) {
        V v{std::forward<Args>(args)...};
        // TODO this is bad!!! We should ask for key/value separately.... and avoid "first"
        auto pos = std::lower_bound(data_.begin(), data_.end(), v, comp_);
        data_.emplace(pos, std::move(v));
    }

    void emplace_back(const V& v) {
        // TODO this is bad!!! We should ask for key/value separately.... and avoid "first"
        auto pos = std::lower_bound(data_.begin(), data_.end(), v, comp_);
        data_.emplace(pos, std::move(v));
    }

    [[nodiscard]] bool empty() const noexcept {
        return data_.empty();
    }

    [[nodiscard]] size_t size() const noexcept {
        return data_.size();
    }

    void reserve(size_t size) noexcept {
        data_.reserve(size);
    }

    constexpr reference front() noexcept {
        return data_.front();
    }

    constexpr const_reference front() const noexcept {
        return data_.front();
    }

    constexpr reference back() noexcept {
        return data_.back();
    }

    constexpr const_reference back() const noexcept {
        return data_.back();
    }

  private:
    std::vector<V> data_;
    Compare comp_;
};

/**
 * A priority queue based on a sorted vector.
 */
template <typename V, typename Compare = std::less<>>
class priority_dequeue {
  public:
    // Member types
    using value_type = V;
    // using allocator_type = Allocator
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using reference = value_type&;
    using const_reference = const value_type&;

  public:
    explicit priority_dequeue(size_t initial_size = 16) noexcept : data_{}, comp_{} {}

    priority_dequeue(const priority_dequeue& rhs) noexcept : data_(rhs.data_), comp_{} {}

    priority_dequeue(priority_dequeue&& rhs) noexcept : data_{std::move(rhs.data_)}, comp_{} {}

    // TODO use default functions?
    // TODO ommit comp_? -> check std::priority_queue
    priority_dequeue& operator=(const priority_dequeue& rhs) noexcept {
        data_ = rhs.data_;
        comp_ = rhs.comp_;
        return *this;
    }

    priority_dequeue& operator=(priority_dequeue&& rhs) noexcept {
        data_ = std::move(rhs.data_);
        comp_ = std::move(rhs.comp_);
        return *this;
    }

    ~priority_dequeue() noexcept = default;

    const V& top() const {
        assert(!data_.empty());
        return data_.back();
    }

    // TODO rename bottom()
    const V& top_max() const {
        assert(!data_.empty());
        return data_.front();
    }

    V& top_max() {
        assert(!data_.empty());
        return data_.front();
    }

    void pop() {
        assert(!data_.empty());
        data_.pop_back();
    }

    void pop_max() {
        assert(!data_.empty());
        data_.pop_front();
    }

    V& operator[](size_t index) noexcept {
        assert(index < data_.size());
        return data_[index];
    }

    const V& operator[](size_t index) const noexcept {
        assert(index < data_.size());
        return data_[index];
    }

    template <typename... Args>
    void emplace(Args&&... args) {
        V v{std::forward<Args>(args)...};
        // TODO this is bad!!! We should ask for key/value separately.... and avoid "first"
        auto pos = std::lower_bound(data_.begin(), data_.end(), v, comp_);
        data_.emplace(pos, std::move(v));
    }

    [[nodiscard]] bool empty() const noexcept {
        return data_.empty();
    }

    [[nodiscard]] size_t size() const noexcept {
        return data_.size();
    }

    void reserve(size_t size) noexcept {
        data_.reserve(size);
    }

    constexpr reference front() noexcept {
        return data_.front();
    }

    constexpr const_reference front() const noexcept {
        return data_.front();
    }

    constexpr reference back() noexcept {
        return data_.back();
    }

    constexpr const_reference back() const noexcept {
        return data_.back();
    }

  private:
    std::deque<V> data_;
    Compare comp_;
};

}  // namespace phtree::bptree::detail

#endif  // PHTREE_COMMON_BPT_PRIORITY_QUEUE_H
