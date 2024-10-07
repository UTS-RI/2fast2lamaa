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

#ifndef PHTREE_COMMON_BASE_TYPES_H
#define PHTREE_COMMON_BASE_TYPES_H

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <limits>
#include <sstream>

/*
 * PLEASE do not include this file directly, it is included via common.h.
 *
 * This file contains specifications for various types used in the PH-Tree, including
 * PhPoint, PhPointD and PhPointBox.
 */
namespace improbable::phtree {

// ************************************************************************
// Constants and base types
// ************************************************************************

using scalar_64_t = int64_t;
using scalar_32_t = int32_t;
using scalar_16_t = int16_t;
using dimension_t = size_t;  // Number of dimensions

namespace detail {
// Bits in a coordinate (usually a double or long has 64 bits, so uint_8 suffices).
// However, uint32_t turned out to be faster, probably due to fewer cycles required for 32bit
// instructions (8bit/16bit tend to require more cycles, see CPU tables available on the web).
using bit_width_t = uint32_t;
// Number of bit for 'scalar_64_t' or 'scalar_32_t'. Note that 'digits' does _not_ include sign bit,
// so e.g. int64_t has 63 `digits`, however we need all bits, i.e. 64.
template <typename SCALAR>
static constexpr bit_width_t MAX_BIT_WIDTH =
    std::numeric_limits<SCALAR>::digits + std::numeric_limits<SCALAR>::is_signed;
// Bit mask
template <typename SCALAR>
using bit_mask_t = typename std::make_unsigned<SCALAR>::type;
template <typename SCALAR>
static constexpr bit_mask_t<SCALAR> MAX_MASK = std::numeric_limits<bit_mask_t<SCALAR>>::max();
// We have two types that represent hypercube addresses (HC position).
// The hc_pos_dim_t uses a template parameter to determine how many bits are needed, this is either
// 32bit or 64bit. This parameter is used where HC positions are stored because benchmarks show a
// difference in performance when this is used.
// The hc_pos_64_t type is always set to 64. It is used where computations play a role that appear
// to prefer being in always 64bit, mainly in CalcPosInArray() and in Node.
template <dimension_t DIM>
using hc_pos_dim_t = std::conditional_t<(DIM < 32), uint32_t, uint64_t>;
using hc_pos_64_t = uint64_t;
}  // namespace detail

// ************************************************************************
// Basic structs and classes
// ************************************************************************

// The SCALAR type needs to be a signet integer, i.e. int32_t or int64_t.
template <dimension_t DIM, typename SCALAR = scalar_64_t>
using PhPoint = std::array<SCALAR, DIM>;

template <dimension_t DIM>
using PhPointD = std::array<double, DIM>;

template <dimension_t DIM>
using PhPointF = std::array<float, DIM>;

template <dimension_t DIM, typename SCALAR = scalar_64_t>
class PhBox {
    using Point = PhPoint<DIM, SCALAR>;

  public:
    explicit PhBox() = default;

    PhBox(const PhBox<DIM, SCALAR>& orig) = default;

    PhBox(const std::array<SCALAR, DIM>& min, const std::array<SCALAR, DIM>& max)
    : min_{min}, max_{max} {}

    [[nodiscard]] const Point& min() const {
        return min_;
    }

    [[nodiscard]] const Point& max() const {
        return max_;
    }

    [[nodiscard]] Point& min() {
        return min_;
    }

    [[nodiscard]] Point& max() {
        return max_;
    }

    void min(const std::array<SCALAR, DIM>& new_min) {
        min_ = new_min;
    }

    void max(const std::array<SCALAR, DIM>& new_max) {
        max_ = new_max;
    }

    auto operator==(const PhBox<DIM, SCALAR>& other) const -> bool {
        return min_ == other.min_ && max_ == other.max_;
    }

    auto operator!=(const PhBox<DIM, SCALAR>& other) const -> bool {
        return min_ != other.min_ || max_ != other.max_;
    }

  private:
    Point min_;
    Point max_;
};

template <dimension_t DIM>
using PhBoxD = PhBox<DIM, double>;

template <dimension_t DIM>
using PhBoxF = PhBox<DIM, float>;

template <dimension_t DIM, typename SCALAR>
std::ostream& operator<<(std::ostream& os, const PhPoint<DIM, SCALAR>& data) {
    assert(DIM >= 1);
    os << "[";
    for (dimension_t i = 0; i < DIM - 1; ++i) {
        os << data[i] << ",";
    }
    os << data[DIM - 1] << "]";
    return os;
}

template <dimension_t DIM, typename SCALAR>
std::ostream& operator<<(std::ostream& os, const PhBox<DIM, SCALAR>& data) {
    os << data.min() << ":" << data.max();
    return os;
}

// Taken from boost::hash_combine
template <class T>
inline void hash_combine(std::size_t& seed, const T& v) {
    seed ^= std::hash<T>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

}  // namespace improbable::phtree

namespace std {
template <improbable::phtree::dimension_t DIM, typename SCALAR>
struct hash<improbable::phtree::PhPoint<DIM, SCALAR>> {
    size_t operator()(const improbable::phtree::PhPoint<DIM, SCALAR>& x) const {
        std::size_t hash_val = 0;
        for (improbable::phtree::dimension_t i = 0; i < DIM; ++i) {
            improbable::phtree::hash_combine(hash_val, x[i]);
        }
        return hash_val;
    }
};
template <improbable::phtree::dimension_t DIM, typename SCALAR>
struct hash<improbable::phtree::PhBox<DIM, SCALAR>> {
    size_t operator()(const improbable::phtree::PhBox<DIM, SCALAR>& x) const {
        std::size_t hash_val = 0;
        for (improbable::phtree::dimension_t i = 0; i < DIM; ++i) {
            improbable::phtree::hash_combine(hash_val, x.min()[i]);
            improbable::phtree::hash_combine(hash_val, x.max()[i]);
        }
        return hash_val;
    }
};
}  // namespace std
#endif  // PHTREE_COMMON_BASE_TYPES_H
