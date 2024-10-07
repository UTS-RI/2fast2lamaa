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

#ifndef PHTREE_COMMON_FILTERS_H
#define PHTREE_COMMON_FILTERS_H

#include "converter.h"
#include "distance.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <limits>

namespace improbable::phtree {

/*
 * Any iterator that has a filter defined will traverse nodes or return values if and only if the
 * filter function returns 'true'. The filter functions are called for every node and every entry
 * (note: internally, nodes are also stored in entries, but these entries will be passed to the
 * filter for nodes) that the iterator encounters. By implication, it will never call the filter
 * function for nodes of entries if their respective parent node has already been rejected.
 *
 * There are separate filter functions for nodes and for key/value entries.
 *
 * Every filter needs to provide two functions:
 * - bool IsEntryValid(const PhPoint<DIM>& key, const T& value);
 *   This function is called for every key/value pair that the query encounters. The function
 *   should return 'true' iff the key/value should be added to the query result.
 *   The parameters are the key and value of the key/value pair.
 *   NOTE: WHen using a MultiMap, 'T' becomes the type of the 'bucket', i.e. the type of the
 *   container that holds multiple entries for a given coordinate.
 * - bool IsNodeValid(const PhPoint<DIM>& prefix, int bits_to_ignore);
 *   This function is called for every node that the query encounters. The function should
 *   return 'true' if the node should be traversed and searched for potential results.
 *   The parameters are the prefix of the node and the number of least significant bits of the
 *   prefix that can (and should) be ignored. The bits of the prefix that should be ignored can
 *   have any value.
 *
 * - bool IsBucketEntryValid(const KeyT& key, const ValueT& value);
 *   This is only used/required for MultiMaps, implementations for a normal PhTree are ignored.
 *   In case of a MultiMap, this method is called for every entry in a bucket (see above).
 */

/*
 * The no-op filter is the default filter for the PH-Tree. It always returns 'true'.
 */
struct FilterNoOp {
    /*
     * @param key The key/coordinate of the entry.
     * @param value The value of the entry. For MultiMaps, this is a container of values.
     * @returns This default implementation always returns `true`.
     */
    template <typename KeyT, typename ValueT>
    constexpr bool IsEntryValid(const KeyT& /*key*/, const ValueT& /*value*/) const noexcept {
        return true;
    }

    /*
     * @param prefix The prefix of node. Any coordinate in the nodes shares this prefix.
     * @param bits_to_ignore The number of bits of the prefix that should be ignored because they
     * are NOT the same for all coordinates in the node. For example, assuming 64bit values, if the
     * node represents coordinates that all share the first 10 bits of the prefix, then the value of
     * bits_to_ignore is 64-10=54.
     * @returns This default implementation always returns `true`.
     */
    template <typename KeyT>
    constexpr bool IsNodeValid(const KeyT& /*prefix*/, int /*bits_to_ignore*/) const noexcept {
        return true;
    }

    /*
     * This is checked once for every entry in a bucket. The method is called once a call to
     * 'IsEntryValid` for the same bucket has returned 'true'. A typical implementation
     * simply returns `true` or checks some values of the entry.
     * @param key The key/coordinate of the bucket entry.
     * @param value The value of the entry.
     * @returns This default implementation always returns `true`.
     */
    template <typename KeyT, typename ValueT>
    constexpr bool IsBucketEntryValid(const KeyT& /*key*/, const ValueT& /*value*/) const noexcept {
        return true;
    }
};

/*
 * The AABB filter can be used to query a point tree for an axis aligned bounding box (AABB).
 * The result is equivalent to that of the 'begin_query(...)' function.
 */
template <typename CONVERTER>
class FilterAABB {
    using KeyExternal = typename CONVERTER::KeyExternal;
    using KeyInternal = typename CONVERTER::KeyInternal;
    using ScalarInternal = typename CONVERTER::ScalarInternal;
    static constexpr auto DIM = CONVERTER::DimInternal;

  public:
    FilterAABB(
        const KeyExternal& min_include, const KeyExternal& max_include, const CONVERTER& converter)
    : min_external_{min_include}
    , max_external_{max_include}
    , min_internal_{converter.pre(min_include)}
    , max_internal_{converter.pre(max_include)}
    , converter_{converter} {};

    /*
     * This function allows resizing/shifting the AABB while iterating over the tree.
     */
    void set(const KeyExternal& min_include, const KeyExternal& max_include) {
        min_external_ = min_include;
        max_external_ = max_include;
        min_internal_ = converter_.get().pre(min_include);
        max_internal_ = converter_.get().pre(max_include);
    }

    template <typename T>
    [[nodiscard]] bool IsEntryValid(const KeyInternal& key, const T& /*value*/) const {
        auto point = converter_.get().post(key);
        for (dimension_t i = 0; i < DIM; ++i) {
            if (point[i] < min_external_[i] || point[i] > max_external_[i]) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool IsNodeValid(const KeyInternal& prefix, std::uint32_t bits_to_ignore) const {
        // Let's assume that we always want to traverse the root node (bits_to_ignore == 64)
        if (bits_to_ignore >= (detail::MAX_BIT_WIDTH<ScalarInternal> - 1)) {
            return true;
        }
        ScalarInternal node_min_bits = detail::MAX_MASK<ScalarInternal> << bits_to_ignore;
        ScalarInternal node_max_bits = ~node_min_bits;

        for (dimension_t i = 0; i < DIM; ++i) {
            if ((prefix[i] | node_max_bits) < min_internal_[i] ||
                (prefix[i] & node_min_bits) > max_internal_[i]) {
                return false;
            }
        }
        return true;
    }

  private:
    KeyExternal min_external_;
    KeyExternal max_external_;
    KeyInternal min_internal_;
    KeyInternal max_internal_;
    std::reference_wrapper<const CONVERTER> converter_;
};

/*
 * The sphere filter can be used to query a point tree for a sphere.
 */
template <typename CONVERTER, typename DISTANCE>
class FilterSphere {
    using KeyExternal = typename CONVERTER::KeyExternal;
    using KeyInternal = typename CONVERTER::KeyInternal;
    using ScalarInternal = typename CONVERTER::ScalarInternal;
    static constexpr auto DIM = CONVERTER::DimInternal;

  public:
    template <typename DIST = DistanceEuclidean<CONVERTER::DimExternal>>
    FilterSphere(
        const KeyExternal& center,
        const double radius,
        const CONVERTER& converter,
        DIST&& distance_function = DIST())
    : center_external_{center}
    , center_internal_{converter.pre(center)}
    , radius_{radius}
    , converter_{converter}
    , distance_function_(std::forward<DIST>(distance_function)){}

    template <typename T>
    [[nodiscard]] bool IsEntryValid(const KeyInternal& key, const T&) const {
        KeyExternal point = converter_.get().post(key);
        return distance_function_(center_external_, point) <= radius_;
    }

    /*
     * Calculate whether AABB encompassing all possible points in the node intersects with the
     * sphere.
     */
    [[nodiscard]] bool IsNodeValid(const KeyInternal& prefix, std::uint32_t bits_to_ignore) const {
        // we always want to traverse the root node (bits_to_ignore == 64)

        if (bits_to_ignore >= (detail::MAX_BIT_WIDTH<ScalarInternal> - 1)) {
            return true;
        }

        ScalarInternal node_min_bits = detail::MAX_MASK<ScalarInternal> << bits_to_ignore;
        ScalarInternal node_max_bits = ~node_min_bits;

        KeyInternal closest_in_bounds;
        for (dimension_t i = 0; i < DIM; ++i) {
            // calculate lower and upper bound for dimension for given node
            ScalarInternal lo = prefix[i] & node_min_bits;
            ScalarInternal hi = prefix[i] | node_max_bits;

            // choose value closest to center for dimension
            closest_in_bounds[i] = std::clamp(center_internal_[i], lo, hi);
        }

        KeyExternal closest_point = converter_.get().post(closest_in_bounds);
        return distance_function_(center_external_, closest_point) <= radius_;
    }

  private:
    KeyExternal center_external_;
    KeyInternal center_internal_;
    double radius_;
    std::reference_wrapper<const CONVERTER> converter_;
    DISTANCE distance_function_;
};
// deduction guide
template <
    typename CONV,
    typename DIST = DistanceEuclidean<CONV::DimExternal>,
    typename P = typename CONV::KeyExternal>
FilterSphere(const P&, double, const CONV&, DIST&& fn = DIST()) -> FilterSphere<CONV, DIST>;

/*
 * AABB filter for box keys.
 * It detects all boxes that overlap partially or fully with the query box.
 */
template <typename CONVERTER>
class FilterBoxAABB {
    using KeyInternal = typename CONVERTER::KeyInternal;
    using ScalarInternal = typename CONVERTER::ScalarInternal;
    using QueryPoint = typename CONVERTER::QueryPointExternal;
    using QueryPointInternal = typename CONVERTER::QueryPointInternal;
    static constexpr auto DIM = CONVERTER::DimExternal;

  public:
    FilterBoxAABB(
        const QueryPoint& min_include, const QueryPoint& max_include, const CONVERTER& converter)
    : min_internal_{converter.pre_query(min_include)}
    , max_internal_{converter.pre_query(max_include)}
    , converter_{converter} {};

    /*
     * This function allows resizing/shifting the AABB while iterating over the tree.
     */
    void set(const QueryPoint& min_include, const QueryPoint& max_include) {
        min_internal_ = converter_.get().pre_query(min_include);
        max_internal_ = converter_.get().pre_query(max_include);
    }

    template <typename T>
    [[nodiscard]] bool IsEntryValid(const KeyInternal& key, const T& /*value*/) const {
        for (dimension_t i = 0; i < DIM; ++i) {
            if (key[i + DIM] < min_internal_[i] || key[i] > max_internal_[i]) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool IsNodeValid(const KeyInternal& prefix, std::uint32_t bits_to_ignore) const {
        // Let's assume that we always want to traverse the root node (bits_to_ignore == 64)
        if (bits_to_ignore >= (detail::MAX_BIT_WIDTH<ScalarInternal> - 1)) {
            return true;
        }
        ScalarInternal node_min_bits = detail::MAX_MASK<ScalarInternal> << bits_to_ignore;
        ScalarInternal node_max_bits = ~node_min_bits;

        for (dimension_t i = 0; i < DIM; ++i) {
            if ((prefix[i] | node_max_bits) < min_internal_[i] ||
                (prefix[i + DIM] & node_min_bits) > max_internal_[i]) {
                return false;
            }
        }
        return true;
    }

  private:
    QueryPointInternal min_internal_;
    QueryPointInternal max_internal_;
    std::reference_wrapper<const CONVERTER> converter_;
};

/*
 * The box sphere filter can be used to query a PH-Tree for boxes that intersect with a sphere.
 */
template <typename CONVERTER, typename DISTANCE>
class FilterBoxSphere {
    using KeyInternal = typename CONVERTER::KeyInternal;
    using ScalarInternal = typename CONVERTER::ScalarInternal;
    using QueryPoint = typename CONVERTER::QueryPointExternal;
    using QueryPointInternal = typename CONVERTER::QueryPointInternal;
    static constexpr auto DIM = CONVERTER::DimExternal;

  public:
    template <typename DIST = DistanceEuclidean<DIM>>
    FilterBoxSphere(
        const QueryPoint& center,
        const double radius,
        const CONVERTER& converter,
        DIST&& distance_function = DIST())
    : center_external_{center}
    , center_internal_{converter.pre_query(center)}
    , radius_{radius}
    , converter_{converter}
    , distance_function_(std::forward<DIST>(distance_function)){}

    template <typename T>
    [[nodiscard]] bool IsEntryValid(const KeyInternal& key, const T&) const {
        QueryPointInternal closest_in_bounds;
        for (dimension_t i = 0; i < DIM; ++i) {
            // choose value closest to center for each dimension
            closest_in_bounds[i] = std::clamp(center_internal_[i], key[i], key[i + DIM]);
        }
        QueryPoint closest_point = converter_.get().post_query(closest_in_bounds);
        return distance_function_(center_external_, closest_point) <= radius_;
    }

    /*
     * Calculate whether AABB of all possible points in the node intersects with the sphere.
     */
    [[nodiscard]] bool IsNodeValid(const KeyInternal& prefix, std::uint32_t bits_to_ignore) const {
        // we always want to traverse the root node (bits_to_ignore == 64)

        if (bits_to_ignore >= (detail::MAX_BIT_WIDTH<ScalarInternal> - 1)) {
            return true;
        }

        ScalarInternal node_min_bits = detail::MAX_MASK<ScalarInternal> << bits_to_ignore;
        ScalarInternal node_max_bits = ~node_min_bits;

        QueryPointInternal closest_in_bounds;
        for (dimension_t i = 0; i < DIM; ++i) {
            // calculate lower and upper bound for dimension for given node
            ScalarInternal lo = prefix[i] & node_min_bits;
            ScalarInternal hi = prefix[i + DIM] | node_max_bits;

            // choose value closest to center for dimension
            closest_in_bounds[i] = std::clamp(center_internal_[i], lo, hi);
        }

        QueryPoint closest_point = converter_.get().post_query(closest_in_bounds);
        return distance_function_(center_external_, closest_point) <= radius_;
    }

  private:
    QueryPoint center_external_;
    QueryPointInternal center_internal_;
    double radius_;
    std::reference_wrapper<const CONVERTER> converter_;
    DISTANCE distance_function_;
};
// deduction guide
template <
    typename CONV,
    typename DIST = DistanceEuclidean<CONV::DimExternal>,
    typename P = typename CONV::KeyExternal>
FilterBoxSphere(const P&, double, const CONV&, DIST&& fn = DIST()) -> FilterBoxSphere<CONV, DIST>;

/*
 * AABB filter for MultiMaps.
 */
template <typename CONVERTER>
class FilterMultiMapAABB : public FilterAABB<CONVERTER> {
    using Key = typename CONVERTER::KeyExternal;
    using KeyInternal = typename CONVERTER::KeyInternal;

  public:
    FilterMultiMapAABB(const Key& min_include, const Key& max_include, CONVERTER& converter)
    : FilterAABB<CONVERTER>(min_include, max_include, converter){};

    template <typename ValueT>
    [[nodiscard]] inline bool IsBucketEntryValid(const KeyInternal&, const ValueT&) const noexcept {
        return true;
    }
};

/*
 * Sphere filter for MultiMaps.
 */
template <typename CONVERTER, typename DISTANCE>
class FilterMultiMapSphere : public FilterSphere<CONVERTER, DISTANCE> {
    using Key = typename CONVERTER::KeyExternal;
    using KeyInternal = typename CONVERTER::KeyInternal;

  public:
    template <typename DIST = DistanceEuclidean<CONVERTER::DimExternal>>
    FilterMultiMapSphere(
        const Key& center, double radius, const CONVERTER& converter, DIST&& dist_fn = DIST())
    : FilterSphere<CONVERTER, DIST>(center, radius, converter, std::forward<DIST>(dist_fn)){}

    template <typename ValueT>
    [[nodiscard]] inline bool IsBucketEntryValid(const KeyInternal&, const ValueT&) const noexcept {
        return true;
    }
};
// deduction guide
template <
    typename CONV,
    typename DIST = DistanceEuclidean<CONV::DimExternal>,
    typename P = typename CONV::KeyExternal>
FilterMultiMapSphere(const P&, double, const CONV&, DIST&& fn = DIST())
    -> FilterMultiMapSphere<CONV, DIST>;

}  // namespace improbable::phtree

#endif  // PHTREE_COMMON_FILTERS_H
