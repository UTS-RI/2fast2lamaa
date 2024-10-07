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

#ifndef PHTREE_PHTREE_H
#define PHTREE_PHTREE_H

#include "common/common.h"
#include "v16/phtree_v16.h"

namespace improbable::phtree {

/*
 * PH-Tree main class.
 * This class is a wrapper which can implement different implementations of the PH-Tree.
 * This class support only `PhPoint` coordinates with `int64_t` scalars.
 *
 * For more information please refer to the README of this project.
 */
template <dimension_t DIM, typename T, typename CONVERTER = ConverterNoOp<DIM, scalar_64_t>>
class PhTree {
    //bit_width_t t;
    friend PhTreeDebugHelper;
    using Key = typename CONVERTER::KeyExternal;
    static constexpr dimension_t DimInternal = CONVERTER::DimInternal;

    // DimInternal==DIM indicates point keys. Box keys have DimInternal==2*DIM.
    using DEFAULT_QUERY_TYPE =
        typename std::conditional<(DIM == DimInternal), QueryPoint, QueryIntersect>::type;

  public:
    // Unless specified otherwise this is just PhBox<DIM, SCALAR_EXTERNAL>
    using QueryBox = typename CONVERTER::QueryBoxExternal;

    template <typename CONV = CONVERTER>
    explicit PhTree(CONV&& converter = CONV()) : tree_{&converter_}, converter_{converter} {}

    PhTree(const PhTree& other) = delete;
    PhTree& operator=(const PhTree& other) = delete;
    PhTree(PhTree&& other) noexcept = default;
    PhTree& operator=(PhTree&& other) noexcept = default;
    ~PhTree() noexcept = default;

    /*
     *  Attempts to build and insert a key and a value into the tree.
     *
     *  @param key The key for the new entry.
     *
     *  @param args  Arguments used to generate a new value.
     *
     *  @return  A pair, whose first element points  to the possibly inserted pair,
     *           and whose second element is a bool that is true if the pair was actually inserted.
     *
     * This function attempts to build and insert a (key, value) pair into the tree. The PH-Tree is
     * effectively a map, so if an entry with the same key was already in the tree, returns that
     * entry instead of inserting a new one.
     */
    template <typename... Args>
    std::pair<T&, bool> emplace(const Key& key, Args&&... args) {
        return tree_.try_emplace(converter_.pre(key), std::forward<Args>(args)...);
    }

    /*
     * The emplace_hint() method uses an iterator as hint for insertion.
     * The hint is ignored if it is not useful or is equal to end().
     *
     * Iterators should normally not be used after the tree has been modified. As an exception to
     * this rule, an iterator can be used as hint if it was previously used with at most one call
     * to erase() and if no other modifications occurred.
     * The following is valid:
     *
     * // Move value from key1 to key2
     * auto iter = tree.find(key1);
     * auto value = iter.second(); // The value may become invalid in erase()
     * erase(iter);
     * emplace_hint(iter, key2, value);  // the iterator can still be used as hint here
     */
    template <typename ITERATOR, typename... Args>
    std::pair<T&, bool> emplace_hint(const ITERATOR& iterator, const Key& key, Args&&... args) {
        return tree_.try_emplace(iterator, converter_.pre(key), std::forward<Args>(args)...);
    }

    /*
     * See std::map::insert().
     *
     * @return a pair consisting of the inserted element (or to the element that prevented the
     * insertion) and a bool denoting whether the insertion took place.
     */
    std::pair<T&, bool> insert(const Key& key, const T& value) {
        return tree_.insert(converter_.pre(key), value);
    }

    /*
     * See emplace().
     */
    template <typename... Args>
    std::pair<T&, bool> try_emplace(const Key& key, Args&&... args) {
        return tree_.try_emplace(converter_.pre(key), std::forward<Args>(args)...);
    }

    /*
     * See emplace_hint().
     */
    template <typename ITERATOR, typename... Args>
    std::pair<T&, bool> try_emplace(const ITERATOR& iterator, const Key& key, Args&&... args) {
        return tree_.try_emplace(iterator, converter_.pre(key), std::forward<Args>(args)...);
    }

    /*
     * @return the value stored at position 'key'. If no such value exists, one is added to the tree
     * and returned.
     */
    T& operator[](const Key& key) {
        return tree_[converter_.pre(key)];
    }

    /*
     * Analogous to map:count().
     *
     * @return '1', if a value is associated with the provided key, otherwise '0'.
     */
    size_t count(const Key& key) const {
        return tree_.count(converter_.pre(key));
    }

    /*
     * Analogous to map:find() except that it returns a NON-ITERABLE iterator. NON-ITERABLE means
     * that the iterator cannot be incremented. It can however be compared to other iterators
     * (like end()) and can obviously be used to access an element.
     * This is an intentional limitation due to performance reasons.
     * To get an _iterable_ iterator, please use `lower_bound()`.
     *
     * Get an entry associated with a k dimensional key.
     * @param key the key to look up
     * @return an NON-ITERABLE "iterator" that points either to the associated value or to
     * {@code end()} if the key was not found.
     */
    auto find(const Key& key) const {
        return tree_.find(converter_.pre(key));
    }

    /*
     * Analogous to map:lower_bound().
     *
     * Get an entry associated with a k dimensional key or the next key.
     * This follows roughly Z-ordering (Morton order), except that negative value come AFTER
     * positive values.
     *
     * @param key the key to look up
     * @return an iterator that points either to the associated value or,
     * if there is no entry with the given key, to the following entry.
     */
    auto lower_bound(const Key& key) const {
        return tree_.lower_bound(converter_.pre(key));
    }

    /*
     * See std::map::erase(). Removes any value associated with the provided key.
     *
     * @return '1' if a value was found, otherwise '0'.
     */
    size_t erase(const Key& key) {
        return tree_.erase(converter_.pre(key));
    }

    /*
     * See std::map::erase(). Removes any entry located at the provided iterator.
     *
     * This function attempts to use the iterator to directly erase the current entry from
     * its node. However, not all iterators provide all required information so this function
     * may resort to erase|(key, value) and thus may not be faster than that.
     *
     * Currently only iterators returned by find(key) will result in faster erase.
     *
     * @return '1' if a value was found, otherwise '0'.
     */
    template <typename ITERATOR>
    size_t erase(const ITERATOR& iterator) {
        return tree_.erase(iterator);
    }

    /*
     * This function attempts to remove a 'value' from 'old_key' and reinsert it for 'new_key'.
     *
     * The function will report _success_ in the following cases:
     * - the value was removed from the old position and reinserted at the new position
     * - the position and new position refer to the same bucket.
     *
     * The function will report _failure_ in the following cases:
     * - The value was already present in the new position
     * - The value was not present in the old position
     *
     * This method will _not_ remove the value from the old position if it is already present at the
     * new position.
     *
     * @param old_key The old position
     * @param new_key The new position
     * @return '1' if the 'value' was moved, otherwise '0'.
     */
    size_t relocate(const Key& old_key, const Key& new_key) {
        return tree_.relocate_if(
            converter_.pre(old_key), converter_.pre(new_key), [](const T&) { return true; });
    }

    /*
     * Relocate (move) an entry from one position to another, subject to a predicate.
     *
     * @param old_key The old position
     * @param new_key The new position
     * @param predicate The predicate is called for every value before it is relocated.
     *                  If the predicate returns 'false', the relocation is aborted.
     * @return '1' if the 'value' was moved, otherwise '0'.
     */
    template <typename PRED>
    size_t relocate_if(const Key& old_key, const Key& new_key, PRED&& predicate) {
        return tree_.relocate_if(
            converter_.pre(old_key), converter_.pre(new_key), std::forward<PRED>(predicate));
    }

    /*
     * Iterates over all entries in the tree. The optional filter allows filtering entries and nodes
     * (=sub-trees) before returning / traversing them. By default all entries are returned. Filter
     * functions must implement the same signature as the default 'FilterNoOp'.
     *
     * @param callback The callback function to be called for every entry that matches the query.
     * The callback requires the following signature: callback(const PhPointD<DIM> &, const T &)
     * @param filter An optional filter function. The filter function allows filtering entries and
     * sub-nodes before they are returned or traversed. Any filter function must follow the
     * signature of the default 'FilterNoOp`.
     */
    template <typename CALLBACK_FN, typename FILTER_FN = FilterNoOp>
    void for_each(CALLBACK_FN&& callback, FILTER_FN&& filter = FILTER_FN()) const {
        tree_.for_each(std::forward<CALLBACK_FN>(callback), std::forward<FILTER_FN>(filter));
    }

    /*
     * Performs a rectangular window query. The parameters are the min and max keys which
     * contain the minimum respectively the maximum keys in every dimension.
     * @param query_box The query window.
     * @param callback The callback function to be called for every entry that matches the query.
     * The callback requires the following signature: callback(const PhPointD<DIM> &, const T &)
     * @param query_type The type of query, such as QueryIntersect or QueryInclude
     * @param filter An optional filter function. The filter function allows filtering entries and
     * sub-nodes before they are returned or traversed. Any filter function must follow the
     * signature of the default 'FilterNoOp`.
     */
    template <
        typename CALLBACK_FN,
        typename FILTER_FN = FilterNoOp,
        typename QUERY_TYPE = DEFAULT_QUERY_TYPE>
    void for_each(
        QueryBox query_box,
        CALLBACK_FN&& callback,
        FILTER_FN&& filter = FILTER_FN(),
        QUERY_TYPE query_type = QUERY_TYPE()) const {
        tree_.for_each(
            query_type(converter_.pre_query(query_box)),
            std::forward<CALLBACK_FN>(callback),
            std::forward<FILTER_FN>(filter));
    }

    /*
     * Iterates over all entries in the tree. The optional filter allows filtering entries and nodes
     * (=sub-trees) before returning / traversing them. By default all entries are returned. Filter
     * functions must implement the same signature as the default 'FilterNoOp'.
     *
     * @return an iterator over all (filtered) entries in the tree,
     */
    template <typename FILTER_FN = FilterNoOp>
    auto begin(FILTER_FN&& filter = FILTER_FN()) const {
        return tree_.begin(std::forward<FILTER_FN>(filter));
    }

    /*
     * Performs a rectangular window query. The parameters are the min and max keys which
     * contain the minimum respectively the maximum keys in every dimension.
     * @param query_box The query window.
     * @param query_type The type of query, such as QueryIntersect or QueryInclude
     * @param filter An optional filter function. The filter function allows filtering entries and
     * sub-nodes before they are returned or traversed. Any filter function must follow the
     * signature of the default 'FilterNoOp`.
     * @return Result iterator.
     */
    template <typename FILTER_FN = FilterNoOp, typename QUERY_TYPE = DEFAULT_QUERY_TYPE>
    auto begin_query(
        const QueryBox& query_box,
        FILTER_FN&& filter = FILTER_FN(),
        QUERY_TYPE query_type = DEFAULT_QUERY_TYPE()) const {
        return tree_.begin_query(
            query_type(converter_.pre_query(query_box)), std::forward<FILTER_FN>(filter));
    }

    /*
     * Locate nearest neighbors for a given point in space.
     *
     * NOTE: This method is not (currently) available for box keys.
     *
     * @param min_results number of entries to be returned. More entries may or may not be returned
     * when several entries have the same distance.
     * @param center center point
     * @param distance_function optional distance function, defaults to euclidean distance
     * @param filter optional filter predicate that excludes nodes/entries before their distance is
     * calculated.
     * @return Result iterator.
     */
    template <
        typename DISTANCE,
        typename FILTER_FN = FilterNoOp,
        // Some magic to disable this in case of box keys, i.e. if DIM != DimInternal
        dimension_t DUMMY = DIM,
        typename std::enable_if<(DUMMY == DimInternal), int>::type = 0>
    auto begin_knn_query(
        size_t min_results,
        const Key& center,
        DISTANCE&& distance_function = DISTANCE(),
        FILTER_FN&& filter = FILTER_FN()) const {
        // We use pre() instead of pre_query() here because, strictly speaking, we want to
        // find the nearest neighbors of a (fictional) key, which may as well be a box.
        return tree_.begin_knn_query(
            min_results,
            converter_.pre(center),
            std::forward<DISTANCE>(distance_function),
            std::forward<FILTER_FN>(filter));
    }

    /*
     * @return An iterator representing the tree's 'end'.
     */
    auto end() const {
        return tree_.end();
    }

    /*
     * Remove all entries from the tree.
     */
    void clear() {
        tree_.clear();
    }

    /*
     * @return the number of entries (key/value pairs) in the tree.
     */
    [[nodiscard]] size_t size() const {
        return tree_.size();
    }

    /*
     * @return 'true' if the tree is empty, otherwise 'false'.
     */
    [[nodiscard]] bool empty() const {
        return tree_.empty();
    }

    /*
     * @return the converter associated with this tree.
     */
    [[nodiscard]] const CONVERTER& converter() const {
        return converter_;
    }

  private:
    // This is used by PhTreeDebugHelper
    const auto& GetInternalTree() const {
        return tree_;
    }

    void CheckConsistencyExternal() const {
        [[maybe_unused]] size_t n = 0;
        for ([[maybe_unused]] const auto& entry : tree_) {
            ++n;
        }
        assert(n == size());
    }

    v16::PhTreeV16<DimInternal, T, CONVERTER> tree_;
    CONVERTER converter_;
};

/**
 * Floating-point `double` version of the PH-Tree.
 * This version of the tree accepts multi-dimensional keys with floating point (`double`)
 * coordinates.
 *
 * The default implementation uses a direct lossless (in terms of numeric precision) mapping from
 * 64bit double to 64bit long integer. The mapping is defined in the Converter functions.
 * Other, lossy mapping have been shown to provide somewhat better performance (due to
 * better tree structure), but this default mapping has been chosen because it is lossless.
 *
 * For more information please refer to the README of this project.
 */
template <dimension_t DIM, typename T, typename CONVERTER = ConverterIEEE<DIM>>
using PhTreeD = PhTree<DIM, T, CONVERTER>;

/**
 * Floating-point `float` version of the PH-Tree.
 * This version of the tree accepts multi-dimensional keys with floating point (`float`)
 * coordinates.
 * See 'PhTreeD' for details.
 */
template <dimension_t DIM, typename T, typename CONVERTER = ConverterFloatIEEE<DIM>>
using PhTreeF = PhTree<DIM, T, CONVERTER>;

/**
 * A PH-Tree that uses (axis aligned) boxes as keys.
 * See 'PhTreeD' for details.
 */
template <dimension_t DIM, typename T, typename CONVERTER_BOX>
using PhTreeBox = PhTree<DIM, T, CONVERTER_BOX>;

/**
 * A PH-Tree that uses (axis aligned) boxes as keys.
 * The boxes are defined with 64bit 'double' floating point coordinates.
 * See 'PhTreeD' for details.
 */
template <dimension_t DIM, typename T, typename CONVERTER_BOX = ConverterBoxIEEE<DIM>>
using PhTreeBoxD = PhTreeBox<DIM, T, CONVERTER_BOX>;

/**
 * A PH-Tree that uses (axis aligned) boxes as keys.
 * The boxes are defined with 32bit 'float' coordinates.
 * See 'PhTreeD' for details.
 */
template <dimension_t DIM, typename T, typename CONVERTER_BOX = ConverterBoxFloatIEEE<DIM>>
using PhTreeBoxF = PhTreeBox<DIM, T, CONVERTER_BOX>;

}  // namespace improbable::phtree

#endif  // PHTREE_PHTREE_H
