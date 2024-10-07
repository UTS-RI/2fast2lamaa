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

#ifndef PHTREE_COMMON_COMMON_H
#define PHTREE_COMMON_COMMON_H

#include "base_types.h"
#include "bits.h"
#include "flat_array_map.h"
#include "flat_sparse_map.h"
#include "tree_stats.h"
#include <cassert>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>

namespace improbable::phtree::detail {

// This is the single-point inclusion file for common types/function/... for the PH-Tree.
// 'single-point inclusion' meaning that including it provides all relevant types/functions/... .

// ************************************************************************
// Bits
// ************************************************************************

/*
 * Encode the bits at the given position of all attributes into a hyper-cube address.
 * Currently, the first attribute determines the left-most (high-value) bit of the address
 * (left to right ordered)
 *
 * @param valSet vector
 * @param postfix_len the postfix length
 * @returns Encoded HC position, which is the index in the array if the entries would be stored in
 * an array.
 */
template <dimension_t DIM, typename SCALAR>
static hc_pos_dim_t<DIM> CalcPosInArray(const PhPoint<DIM, SCALAR>& key, bit_width_t postfix_len) {
    // n=DIM,  i={0..n-1}
    // i = 0 :  |0|1|0|1|0|1|0|1|
    // i = 1 :  | 0 | 1 | 0 | 1 |
    // i = 2 :  |   0   |   1   |
    // len = 2^n
    // Following formula was for inverse ordering of current ordering...
    // pos = sum (i=1..n, len/2^i) = sum (..., 2^(n-i))
    bit_mask_t<SCALAR> mask = bit_mask_t<SCALAR>(1) << postfix_len;
    hc_pos_64_t pos = 0;
    for (dimension_t i = 0; i < DIM; ++i) {
        pos <<= 1;
        // set pos-bit if bit is set in key
        pos |= (mask & key[i]) >> postfix_len;
    }
    return static_cast<hc_pos_dim_t<DIM>>(pos);
}

template <dimension_t DIM, typename SCALAR>
static bool IsInRange(
    const PhPoint<DIM, SCALAR>& key,
    const PhPoint<DIM, SCALAR>& min,
    const PhPoint<DIM, SCALAR>& max) {
    for (dimension_t i = 0; i < DIM; ++i) {
        auto k = key[i];
        if (k < min[i] || k > max[i]) {
            return false;
        }
    }
    return true;
}

/*
 * @param v1 key 1
 * @param v2 key 2
 * @return the number of diverging bits. For each dimension we determine the most significant bit
 * where the two keys differ. We then count this bit plus all trailing bits (even if individual bits
 * may be the same). Then we return the highest number of diverging bits found in any dimension of
 * the two keys. In case of key1==key2 we return 0. In other words, for 64 bit keys, we return 64
 * minus the number of leading bits that are common in both keys across all dimensions.
 */
template <dimension_t DIM, typename SCALAR>
static bit_width_t NumberOfDivergingBits(
    const PhPoint<DIM, SCALAR>& v1, const PhPoint<DIM, SCALAR>& v2) {
    // write all differences to diff, we just check diff afterwards
    SCALAR diff = 0;
    for (dimension_t i = 0; i < DIM; ++i) {
        diff |= (v1[i] ^ v2[i]);
    }
    auto diff2 = reinterpret_cast<bit_mask_t<SCALAR>&>(diff);
    if constexpr (MAX_BIT_WIDTH<SCALAR> > 32) {
        return MAX_BIT_WIDTH<SCALAR> - CountLeadingZeros64(diff2);
    }
    return MAX_BIT_WIDTH<SCALAR> - CountLeadingZeros(diff2);
}

template <dimension_t DIM, typename SCALAR, typename MASK>
void CalcLimits(
    bit_width_t postfix_len,
    const PhPoint<DIM, SCALAR>& range_min,
    const PhPoint<DIM, SCALAR>& range_max,
    const PhPoint<DIM, SCALAR>& prefix,
    MASK& mask_lower,
    MASK& mask_upper) {
    // create limits for the local node. there is a lower and an upper limit. Each limit
    // consists of a series of DIM bit, one for each dimension.
    // For the lower limit, a '1' indicates that the 'lower' half of this dimension does
    // not need to be queried.
    // For the upper limit, a '0' indicates that the 'higher' half does not need to be
    // queried.
    //
    //              ||  lower_limit=0 || lower_limit=1 || upper_limit = 0 || upper_limit = 1
    // =============||======================================================================
    // query lower  ||     YES              NO
    // ============ || =====================================================================
    // query higher ||                                       NO                  YES
    //
    assert(postfix_len < MAX_BIT_WIDTH<SCALAR>);
    mask_lower = 0;
    mask_upper = 0;
    // to prevent problems with signed long when using 64 bit
    if (postfix_len < MAX_BIT_WIDTH<SCALAR> - 1) {
        for (dimension_t i = 0; i < DIM; ++i) {
            mask_lower <<= 1;
            //==> set to 1 if lower value should not be queried
            mask_lower |= range_min[i] >= prefix[i];
        }
        for (dimension_t i = 0; i < DIM; ++i) {
            mask_upper <<= 1;
            // Leave 0 if higher value should not be queried.
            mask_upper |= range_max[i] >= prefix[i];
        }
    } else {
        // special treatment for signed longs
        // The problem (difference) here is that a '1' at the leading bit does indicate a
        // LOWER value, opposed to indicating a HIGHER value as in the remaining 63 bits.
        // The hypercube assumes that a leading '0' indicates a lower value.
        // Solution: We leave HC as it is.
        for (dimension_t i = 0; i < DIM; ++i) {
            mask_upper <<= 1;
            // If minimum is positive, we don't need the search negative values
            //==> set upper_limit to 0, prevent searching values starting with '1'.
            mask_upper |= range_min[i] < 0;
        }
        for (dimension_t i = 0; i < DIM; ++i) {
            mask_lower <<= 1;
            // Leave 0 if higher value should not be queried
            // If maximum is negative, we do not need to search positive values
            //(starting with '0').
            //--> lower_limit = '1'
            mask_lower |= range_max[i] < 0;
        }
    }
}

template <dimension_t DIM, typename SCALAR>
static bool KeyEquals(
    const PhPoint<DIM, SCALAR>& key_a, const PhPoint<DIM, SCALAR>& key_b, bit_width_t ignore_bits) {
    SCALAR diff{0};
    for (dimension_t i = 0; i < DIM; ++i) {
        diff |= key_a[i] ^ key_b[i];
    }
    return diff >> ignore_bits == 0;
}

/**
 * @return "true" iff the first key comes before the second key when using Morton order
 * (z-ordering). Exception: negative values are considered "larger" than positive values due to
 * their first bit being always '1'.
 */
template <dimension_t DIM, typename SCALAR>
bool KeyLess(const PhPoint<DIM, SCALAR>& k1, const PhPoint<DIM, SCALAR>& k2) {
    auto b = NumberOfDivergingBits(k1, k2);
    return b > 0 && CalcPosInArray(k1, b - 1) < CalcPosInArray(k2, b - 1);
}

// ************************************************************************
// String helpers
// ************************************************************************

template <typename SCALAR>
static inline std::string ToBinary(SCALAR l, bit_width_t width = MAX_BIT_WIDTH<SCALAR>) {
    std::ostringstream sb;
    // long mask = DEPTH < 64 ? (1<<(DEPTH-1)) : 0x8000000000000000L;
    for (bit_width_t i = 0; i < width; ++i) {
        bit_mask_t<SCALAR> mask = (bit_mask_t<SCALAR>(1) << (width - i - 1));
        sb << ((l & mask) != 0 ? "1" : "0");
        if ((i + 1) % 8 == 0 && (i + 1) < width) {
            sb << '.';
        }
    }
    return sb.str();
}

template <dimension_t DIM, typename SCALAR>
static inline std::string ToBinary(
    const PhPoint<DIM, SCALAR>& la, bit_width_t width = MAX_BIT_WIDTH<SCALAR>) {
    std::ostringstream sb;
    for (dimension_t i = 0; i < DIM; ++i) {
        sb << ToBinary(la[i], width) << ", ";
    }
    return sb.str();
}

}  // namespace improbable::phtree::detail

#endif  // PHTREE_COMMON_COMMON_H
