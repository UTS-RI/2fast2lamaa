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

#ifndef PHTREE_COMMON_BPT_FIXED_VECTOR_H
#define PHTREE_COMMON_BPT_FIXED_VECTOR_H

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <tuple>

namespace phtree::bptree::detail {

template <typename V>
class bpt_vector_iterator {
  private:
    using normal_iterator = bpt_vector_iterator<V>;

  public:
    using iterator_category = std::random_access_iterator_tag;
    using value_type = V;
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;

    bpt_vector_iterator() noexcept : ptr_{nullptr} {}
    explicit bpt_vector_iterator(value_type* ptr) noexcept : ptr_{ptr} {}

    reference operator*() const noexcept {
        return *ptr_;
    }

    pointer operator->() const noexcept {
        return ptr_;
    }

    constexpr bpt_vector_iterator& operator++() noexcept {
        ++ptr_;
        return *this;
    }

    constexpr bpt_vector_iterator operator++(int) noexcept {
        return bpt_vector_iterator(ptr_++);
    }

    constexpr bool operator<(const bpt_vector_iterator<V>& right) const noexcept {
        return ptr_ < right.ptr_;
    }

    friend bool operator==(
        const bpt_vector_iterator<V>& left, const bpt_vector_iterator<V>& right) noexcept {
        return left.ptr_ == right.ptr_;
    }

    friend bool operator!=(
        const bpt_vector_iterator<V>& left, const bpt_vector_iterator<V>& right) noexcept {
        return left.ptr_ != right.ptr_;
    }

    // Bidirectional iterator requirements
    constexpr normal_iterator& operator--() noexcept {
        --ptr_;
        return *this;
    }

    constexpr normal_iterator operator--(int) noexcept {
        return normal_iterator(ptr_--);
    }

    // Random access iterator requirements
    constexpr reference operator[](difference_type n) const noexcept {
        return ptr_[n];
    }

    constexpr normal_iterator& operator+=(difference_type n) noexcept {
        ptr_ += n;
        return *this;
    }

    constexpr normal_iterator operator+(difference_type n) const noexcept {
        return normal_iterator(ptr_ + n);
    }

    constexpr normal_iterator& operator-=(difference_type n) noexcept {
        ptr_ -= n;
        return *this;
    }

    constexpr normal_iterator operator-(difference_type n) const noexcept {
        return normal_iterator(ptr_ - n);
    }

    // Other // TODO???
    constexpr auto operator-(const normal_iterator& it) const noexcept {
        return ptr_ - it.ptr_;
    }

    //    constexpr normal_iterator operator-(V* ptr) const noexcept {
    //        return normal_iterator(ptr_ - ptr);
    //    }

    // implicit conversion to const iterator
    operator bpt_vector_iterator<const V>() {
        return bpt_vector_iterator<const V>{ptr_};
    }

  private:
    V* ptr_;
};

template <typename V, size_t SIZE = 16>
class bpt_vector {
  public:
    // TODO implement "Member types":  https://en.cppreference.com/w/cpp/container/vector
  public:
    // Member types
    using value_type = V;
    // using allocator_type = Allocator
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using reference = value_type&;
    using const_reference = const value_type&;
    using pointer = value_type*;  //	Allocator::pointer 	(until C++11)
    //     std::allocator_traits<Allocator>::pointer 	(since C++11)
    using const_pointer = const value_type*;  //	Allocator::const_pointer 	(until C++11)
    //             std::allocator_traits<Allocator>::const_pointer 	(since C++11)
    // TODO LegacyContiguousIterator ?!!?!?!?
    // using iterator = bpt_vector_iterator<value_type>;  // LegacyRandomAccessIterator
    // using const_iterator = bpt_vector_iterator<const value_type>;
    using iterator = value_type*;  // LegacyRandomAccessIterator
    using const_iterator = const value_type*;

    ~bpt_vector() noexcept {
        std::destroy_n(begin(), size_);
    }

    constexpr iterator begin() noexcept {
        return to_iter(0);
    }

    constexpr const_iterator begin() const noexcept {
        return to_iter_c(0);
    }

    constexpr iterator end() noexcept {
        return to_iter(size_);
    }

    constexpr const_iterator end() const noexcept {
        return to_iter_c(size_);
    }

    reference front() noexcept {
        return data(0);
    }

    const_reference front() const noexcept {
        return data_c(0);
    }

    reference back() noexcept {
        return data(size_ - 1);
    }

    const_reference back() const noexcept {
        return data_c(size_ - 1);
    }

    reference operator[](size_t index) noexcept {
        assert(index < size_);
        return data(index);
    }

    const_reference operator[](size_t index) const noexcept {
        assert(index < size_);
        return data_c(index);
    }

    template <typename InputIterator>
    iterator insert(const_iterator iter, InputIterator src_begin, InputIterator src_end) {
        auto length = src_end - src_begin;
        assert(size_ + length <= SIZE);
        auto index = to_index(iter);

        pointer dst_ptr = {const_cast<pointer>(&*iter)};
        move_to_back(dst_ptr + length, dst_ptr, size_ - index);

        // TODO Use memmove if the source is also a bpt_vector....
        // if constexpr (src_begin.base() is b_vector_iterator) ... move_to_back else loop
        auto src = src_begin;
        while (src != src_end) {
            place(dst_ptr, std::move(*src));
            ++src;
            ++dst_ptr;
        }
        size_ += length;
        assert(size_ <= SIZE);
        return begin() + index;
    }

    template <typename... Args>
    iterator emplace(const_iterator iter, Args&&... args) {
        assert(size_ < SIZE);
        auto index = to_index(iter);
        pointer dst_ptr{const_cast<pointer>(&*iter)};
        move_to_back(dst_ptr + 1, dst_ptr, size_ - index);
        place(dst_ptr, std::forward<Args>(args)...);
        ++size_;
        return iterator{dst_ptr};
    }

    template <typename... Args>
    reference emplace_back(Args&&... args) {
        assert(size_ < SIZE);
        return *place(size_++, std::forward<Args>(args)...);
    }

    iterator erase(const_iterator iter) noexcept {
        std::destroy_at(&*iter);
        pointer dst{const_cast<pointer>(&*iter)};
        move_to_front(dst, dst + 1, size_ - to_index(iter) - 1);
        --size_;
        return iterator{dst};
    }

    iterator erase(const_iterator first, const_iterator last) noexcept {
        std::destroy(first, last);
        auto length = last - first;
        pointer dst{const_cast<pointer>(&*first)};
        move_to_front(dst, dst + length, end_ptr() - &*last);
        size_ -= length;
        return iterator{dst};
    }

    [[nodiscard]] size_t size() const noexcept {
        return size_;
    }

    [[nodiscard]] bool empty() const noexcept {
        return size_ == 0;
    }

    void reserve(size_t) noexcept {}

  private:
    template <class V2 = V, typename std::enable_if_t<std::is_trivially_copyable_v<V2>, int> = 0>
    void move_to_back(pointer dst, pointer src, size_t count) noexcept {
        memmove(dst, src, count * sizeof(V));
    }

    template <class V2 = V, typename std::enable_if_t<!std::is_trivially_copyable_v<V2>, int> = 0>
    void move_to_back(pointer dst, pointer src, size_t count) noexcept {
        if (count > 0) {
            dst += count - 1;
            src += count - 1;
            for (size_t i = 0; i < count; ++i, --dst, --src) {
                data_ptr(dst) = std::move(data_ptr(src));
            }
        }
    }

    template <class V2 = V, typename std::enable_if_t<std::is_trivially_copyable_v<V2>, int> = 0>
    void move_to_front(pointer dst, pointer src, size_t count) noexcept {
        memmove(dst, src, count * sizeof(V));
    }

    template <class V2 = V, typename std::enable_if_t<!std::is_trivially_copyable_v<V2>, int> = 0>
    void move_to_front(pointer dst, pointer src, size_t count) noexcept {
        for (size_t i = 0; i < count; ++i, ++dst, ++src) {
            *dst = std::move(*src);
        }
    }

    template <typename... Args>
    pointer place(size_t index, Args&&... args) noexcept {
        return new (reinterpret_cast<void*>(&data_[index * sizeof(V)]))
            V{std::forward<Args>(args)...};
    }

    template <typename... Args>
    pointer place(pointer ptr, Args&&... args) noexcept {
        return new (reinterpret_cast<void*>(ptr)) V{std::forward<Args>(args)...};
    }

    size_type to_index(const_iterator& iter) const noexcept {
        return &*iter - reinterpret_cast<const_pointer>(&data_);
    }

    const_iterator to_iter_c(size_t index) const noexcept {
        return iterator{&data(index)};
    }

    iterator to_iter(size_t index) noexcept {
        return iterator{&data(index)};
    }

    reference data_ptr(pointer ptr) noexcept {
        return *std::launder(ptr);
    }

    reference data(size_t index) noexcept {
        return *std::launder(reinterpret_cast<V*>(&data_[index * sizeof(V)]));
    }

    const_reference data_c(size_t index) const noexcept {
        return *std::launder(reinterpret_cast<const V*>(&data_[index * sizeof(V)]));
    }

    pointer end_ptr() noexcept {
        return std::launder(reinterpret_cast<V*>(&data_[size_ * sizeof(V)]));
    }

    // We use an untyped array to avoid implicit calls to constructors and destructors of entries.
    alignas(V) std::byte data_[sizeof(V) * SIZE];
    size_t size_{0};
};
}  // namespace phtree::bptree::detail

#endif  // PHTREE_COMMON_BPT_FIXED_VECTOR_H
