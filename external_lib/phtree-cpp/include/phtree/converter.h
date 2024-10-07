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

#ifndef PHTREE_COMMON_CONVERTER_H
#define PHTREE_COMMON_CONVERTER_H

#include "common/common.h"
#include <cstring>

/*
 * PLEASE do not include this file directly, it is included via common.h.
 *
 * This file contains conversion/transformation functions for converting user coordinates and
 * shapes, such as PhPointD and PhBoxD, into PH-Tree native coordinates (PhPoint).
 */
namespace improbable::phtree {

class ScalarConverterIEEE {
    static_assert(std::is_same<scalar_64_t, std::int64_t>());

  public:
    static scalar_64_t pre(double value) {
        // To create a sortable long, we convert the double to a long using the IEEE-754 standard,
        // which stores floats in the form <sign><exponent-127><mantissa> .
        // This result is properly ordered longs for all positive doubles. Negative values have
        // inverse ordering. For negative doubles, we therefore simply invert them to make them
        // sortable, however the sign must be inverted again to stay negative.
        // Also, we could use reinterpret_cast, but that fails on GCC. Using memcpy results in the
        // same asm instructions as reinterpret_cast().
        scalar_64_t r;
        memcpy(&r, &value, sizeof(r));
        return r >= 0 ? r : r ^ 0x7FFFFFFFFFFFFFFFL;
    }

    static double post(scalar_64_t value) {
        auto v = value >= 0 ? value : value ^ 0x7FFFFFFFFFFFFFFFL;
        double r;
        memcpy(&r, &v, sizeof(r));
        return r;
    }

    static scalar_32_t pre(float value) {
        // To create a sortable long, we convert the double to a long using the IEEE-754 standard,
        // which stores floats in the form <sign><exponent-127><mantissa> .
        // This result is properly ordered longs for all positive doubles. Negative values have
        // inverse ordering. For negative doubles, we therefore simply invert them to make them
        // sortable, however the sign must be inverted again to stay negative.
        scalar_32_t r;
        memcpy(&r, &value, sizeof(r));
        return r >= 0 ? r : r ^ 0x7FFFFFFFL;
    }

    static float post(scalar_32_t value) {
        auto v = value >= 0 ? value : value ^ 0x7FFFFFFFL;
        float r;
        memcpy(&r, &v, sizeof(r));
        return r;
    }
};

/*
 * The ScalarMultiplyConverter converts floating point scalars 'f' (double or float) to integral
 * scalars 'i' and back.
 * Unlike other scalar converters, the multiply-converter does this by multiplying the floating
 * point scalar with a fraction such that "i = (cast to integral) f * NUMERATOR/DENOMINATOR".
 *
 * Warning: This conversion is inherently lossy due to the cast to an integral type.
 * Converting a value f1 to i and then back to f2 will likely result in f1 != f2.
 */
template <int64_t NUMERATOR, int64_t DENOMINATOR>
class ScalarConverterMultiply {
    static_assert(std::is_same<scalar_64_t, std::int64_t>());
    static_assert(NUMERATOR != 0);
    static_assert(DENOMINATOR != 0);
    static constexpr double MULTIPLY = NUMERATOR / (double)DENOMINATOR;
    static constexpr double DIVIDE = DENOMINATOR / (double)NUMERATOR;

  public:
    static scalar_64_t pre(double value) {
        return static_cast<scalar_64_t>(value * MULTIPLY);
    }

    static double post(scalar_64_t value) {
        return value * DIVIDE;
    }

    static scalar_32_t pre(float value) {
        return  static_cast<scalar_32_t>(value * MULTIPLY);
    }

    static float post(scalar_32_t value) {
        return value * DIVIDE;
    }
};

/*
 * Converters convert points and boxes using a ScalarConverter.
 * The common base class provides type information for users of the converters.
 */
template <
    dimension_t DIM_EXTERNAL,
    dimension_t DIM_INTERNAL,
    typename SCALAR_EXTERNAL,
    typename SCALAR_INTERNAL,
    typename KEY_EXTERNAL,
    typename QUERY_POINT_EXTERNAL = PhBox<DIM_EXTERNAL, SCALAR_EXTERNAL>>
class ConverterBase {
  public:
    static constexpr dimension_t DimExternal = DIM_EXTERNAL;
    static constexpr dimension_t DimInternal = DIM_INTERNAL;
    using ScalarExternal = SCALAR_EXTERNAL;
    using ScalarInternal = SCALAR_INTERNAL;
    using KeyExternal = KEY_EXTERNAL;
    using KeyInternal = PhPoint<DIM_INTERNAL, SCALAR_INTERNAL>;
    using QueryBoxExternal = QUERY_POINT_EXTERNAL;
    using QueryBoxInternal = PhBox<DIM_EXTERNAL, SCALAR_INTERNAL>;
    using QueryPointExternal = PhPoint<DIM_EXTERNAL, SCALAR_EXTERNAL>;
    using QueryPointInternal = PhPoint<DIM_EXTERNAL, SCALAR_INTERNAL>;
};

/*
 * Common base class for converters for point keys.
 * This class exists only as convenience for developing custom converters.
 */
template <dimension_t DIM, typename SCALAR_EXTERNAL, typename SCALAR_INTERNAL>
using ConverterPointBase =
    ConverterBase<DIM, DIM, SCALAR_EXTERNAL, SCALAR_INTERNAL, PhPoint<DIM, SCALAR_EXTERNAL>>;

/*
 * Common base class for converters for box keys.
 * This class exists only as convenience for developing custom converters.
 */
template <dimension_t DIM, typename SCALAR_EXTERNAL, typename SCALAR_INTERNAL>
using ConverterBoxBase =
    ConverterBase<DIM, 2 * DIM, SCALAR_EXTERNAL, SCALAR_INTERNAL, PhBox<DIM, SCALAR_EXTERNAL>>;

template <dimension_t DIM, typename SCALAR>
struct ConverterNoOp : public ConverterPointBase<DIM, SCALAR, SCALAR> {
    using BASE = ConverterPointBase<DIM, SCALAR, SCALAR>;
    using Point = typename BASE::KeyExternal;
    using PointInternal = typename BASE::KeyInternal;

    constexpr const PointInternal& pre(const Point& point) const {
        return point;
    }

    constexpr const Point& post(const PointInternal& point) const {
        return point;
    }

    constexpr const PhBox<DIM, SCALAR>& pre_query(const PhBox<DIM, SCALAR>& box) const {
        return box;
    }
};

/*
 * Simple point converter that treats all dimensions the same way.
 */
template <
    dimension_t DIM,
    typename SCALAR_EXTERNAL,
    typename SCALAR_INTERNAL,
    typename CONVERT = ScalarConverterIEEE>
class SimplePointConverter : public ConverterPointBase<DIM, SCALAR_EXTERNAL, SCALAR_INTERNAL> {
    using BASE = ConverterPointBase<DIM, SCALAR_EXTERNAL, SCALAR_INTERNAL>;

  public:
    using Point = typename BASE::KeyExternal;
    using PointInternal = typename BASE::KeyInternal;
    using QueryBox = typename BASE::QueryBoxExternal;

    static_assert(std::is_same<Point, PhPoint<DIM, SCALAR_EXTERNAL>>::value);
    static_assert(std::is_same<PointInternal, PhPoint<DIM, SCALAR_INTERNAL>>::value);

  public:
    explicit SimplePointConverter(const CONVERT converter = CONVERT()) : converter_{converter} {};

    PointInternal pre(const Point& point) const {
        PointInternal out;
        for (dimension_t i = 0; i < DIM; ++i) {
            out[i] = converter_.pre(point[i]);
        }
        return out;
    }

    Point post(const PointInternal& point) const {
        Point out;
        for (dimension_t i = 0; i < DIM; ++i) {
            out[i] = converter_.post(point[i]);
        }
        return out;
    }

    PhBox<DIM, SCALAR_INTERNAL> pre_query(const QueryBox& query_box) const {
        return {pre(query_box.min()), pre(query_box.max())};
    }

  private:
    CONVERT converter_;
};

template <
    dimension_t DIM,
    typename SCALAR_EXTERNAL,
    typename SCALAR_INTERNAL,
    typename CONVERT = ScalarConverterIEEE>
class SimpleBoxConverter : public ConverterBoxBase<DIM, SCALAR_EXTERNAL, SCALAR_INTERNAL> {
    using BASE = ConverterBoxBase<DIM, SCALAR_EXTERNAL, SCALAR_INTERNAL>;

  public:
    using Box = typename BASE::KeyExternal;
    using PointInternal = typename BASE::KeyInternal;
    using QueryBox = typename BASE::QueryBoxExternal;
    using QueryBoxInternal = typename BASE::QueryBoxInternal;
    using QueryPoint = typename BASE::QueryPointExternal;
    using QueryPointInternal = typename BASE::QueryPointInternal;

    static_assert(std::is_same<Box, PhBox<DIM, SCALAR_EXTERNAL>>::value);
    static_assert(std::is_same<PointInternal, PhPoint<2 * DIM, SCALAR_INTERNAL>>::value);

  public:
    explicit SimpleBoxConverter(const CONVERT converter = CONVERT()) : converter_{converter} {};
    PointInternal pre(const Box& box) const {
        PointInternal out;
        for (dimension_t i = 0; i < DIM; ++i) {
            out[i] = converter_.pre(box.min()[i]);
            out[i + DIM] = converter_.pre(box.max()[i]);
        }
        return out;
    }

    Box post(const PointInternal& point) const {
        Box out;
        for (dimension_t i = 0; i < DIM; ++i) {
            out.min()[i] = converter_.post(point[i]);
            out.max()[i] = converter_.post(point[i + DIM]);
        }
        return out;
    }

    auto pre_query(const QueryBox& query_box) const {
        QueryBoxInternal out;
        auto& min = out.min();
        auto& max = out.max();
        for (dimension_t i = 0; i < DIM; ++i) {
            min[i] = converter_.pre(query_box.min()[i]);
            max[i] = converter_.pre(query_box.max()[i]);
        }
        return out;
    }

    auto pre_query(const QueryPoint& query_point) const {
        QueryPointInternal out;
        for (dimension_t i = 0; i < DIM; ++i) {
            out[i] = converter_.pre(query_point[i]);
        }
        return out;
    }

    auto post_query(const QueryPointInternal& query_point) const {
        QueryPoint out;
        for (dimension_t i = 0; i < DIM; ++i) {
            out[i] = converter_.post(query_point[i]);
        }
        return out;
    }

  private:
    CONVERT converter_;
};

/*
 * IEEE Converters convert float/double to integral types using some bit operations.
 * The conversion maintain ordering and precision (i.e. it is a loss-less conversion).
 * However, it results in a strongly non-metric distortion of the space, so distances between
 * converted coordinates cannot easily be converted back to normal double/float metric space.
 */
template <dimension_t DIM>
using ConverterIEEE = SimplePointConverter<DIM, double, scalar_64_t, ScalarConverterIEEE>;

template <dimension_t DIM>
using ConverterFloatIEEE = SimplePointConverter<DIM, float, scalar_32_t, ScalarConverterIEEE>;

template <dimension_t DIM>
using ConverterBoxIEEE = SimpleBoxConverter<DIM, double, scalar_64_t, ScalarConverterIEEE>;

template <dimension_t DIM>
using ConverterBoxFloatIEEE = SimpleBoxConverter<DIM, float, scalar_32_t, ScalarConverterIEEE>;

/*
 * Multiply Converter.
 * The multiply converter converts float/double to integral type by multiplying them
 * with NUMERATOR/DENOMINATOR and then casting them to the desired integral type.
 * This conversion may lossy, i.e. values may lose precision when converted with this converter.
 */

template <dimension_t DIM, int64_t NUMERATOR, int64_t DENOMINATOR>
using ConverterMultiply =
    SimplePointConverter<DIM, double, scalar_64_t, ScalarConverterMultiply<NUMERATOR, DENOMINATOR>>;

template <dimension_t DIM, int64_t NUMERATOR, int64_t DENOMINATOR>
using ConverterBoxMultiply =
    SimpleBoxConverter<DIM, double, scalar_64_t, ScalarConverterMultiply<NUMERATOR, DENOMINATOR>>;

/*
 * The behaviour of window-queries on Ph-Trees with box keys can be configured with QueryTypes.
 * For example, a window query of type QueryIntersect will return all keys that have at least
 * some overlap with the query window. Queries of type QueryInclude will only return keys that
 * completely overlap with (= are included in) the query window.
 *
 * The query types (QueryIntersect etc) take as argument a pair of points or a box that represent
 * an axis aligned rectangular query window. The input point need to have been transformed already
 * into the tree's internal coordinate system with the respective converter.
 *
 * The output of the query type functions is a pair of box keys.
 * The meaning of these boxes is somewhat unintuitive, but they represent the min/max coordinates
 * in the tree's internal representation where boxes are stored as points.
 *
 * Default implementations:
 * - QueryPoint is a no-op implementation for point keys
 *   (no-op because intersect/include make no sense for points)
 * - QueryIntersect returns all box keys that intersects with the query box
 * - QueryInclude returns all box keys that lies completely inside the query box
 */

/**
 * No-op query-type for querying a PH-Tree that contains point keys.
 */
struct QueryPoint {
    template <dimension_t DIM, typename SCALAR_INTERNAL>
    auto operator()(const PhBox<DIM, SCALAR_INTERNAL>& query_box) {
        return query_box;
    }
};

/*
 * The function produces a pair of 2*DIM points that can be used to perform an 'intersect'
 * type query on a PhTree that contains box data. I.e. the query will return any boxes that
 * intersect with the original query window.
 * Here 'intersect' includes any boxes with marginal overlap, i.e. any box that has at least
 * one point in common with the query window.
 * For example a box {{1,1},{3,3}} is considered to overlap with a box {{3,3},{5,5}}.
 */
struct QueryIntersect {
    template <dimension_t DIM, typename SCALAR_INTERNAL>
    auto operator()(const PhBox<DIM, SCALAR_INTERNAL>& query_box) {
        auto neg_inf = std::numeric_limits<SCALAR_INTERNAL>::min();
        auto pos_inf = std::numeric_limits<SCALAR_INTERNAL>::max();
        PhBox<2 * DIM, SCALAR_INTERNAL> min_max;
        auto& min = min_max.min();
        auto& max = min_max.max();
        for (dimension_t i = 0; i < DIM; i++) {
            min[i] = neg_inf;
            min[i + DIM] = query_box.min()[i];
            max[i] = query_box.max()[i];
            max[i + DIM] = pos_inf;
        }
        return min_max;
    }
};

/*
 * QueryInclude set up a query that return only key that are completely included in a rectangular
 * query window defined by min/max (inclusive).
 * For example, a box {{1,1},{3,3}} is considered to be included in a box {{1,1},{5,5}}.
 */
struct QueryInclude {
    template <dimension_t DIM, typename SCALAR_INTERNAL>
    auto operator()(const PhBox<DIM, SCALAR_INTERNAL>& query_box) {
        PhBox<2 * DIM, SCALAR_INTERNAL> min_max;
        auto& min = min_max.min();
        auto& max = min_max.max();
        for (dimension_t i = 0; i < DIM; i++) {
            min[i] = query_box.min()[i];
            min[i + DIM] = query_box.min()[i];
            max[i] = query_box.max()[i];
            max[i + DIM] = query_box.max()[i];
        }
        return min_max;
    }
};
}  // namespace improbable::phtree

#endif  // PHTREE_COMMON_CONVERTER_H
