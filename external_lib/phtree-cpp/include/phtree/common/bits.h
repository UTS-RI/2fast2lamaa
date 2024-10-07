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

#ifndef PHTREE_COMMON_BITS_H
#define PHTREE_COMMON_BITS_H

#include "base_types.h"
#include <cassert>

#if defined(_MSC_VER)
// https://docs.microsoft.com/en-us/cpp/intrinsics/x64-amd64-intrinsics-list?view=vs-2019
#include <intrin.h>
#endif

/*
 * PLEASE do not include this file directly, it is included via common.h.
 *
 * This file defines how certain bit level operations are implemented, such as:
 * - count leading zeroes
 * - count trailing zeros
 */
namespace improbable::phtree::detail {

#if defined(__clang__) || defined(__GNUC__)
// See https://en.cppreference.com/w/cpp/language/types
inline bit_width_t CountLeadingZeros64(std::uint64_t bit_string) {
    return bit_string == 0 ? 64U : __builtin_clzll(bit_string);
}

inline bit_width_t CountLeadingZeros(std::uint32_t bit_string) {
    return bit_string == 0 ? 32U : __builtin_clz(bit_string);
}

inline bit_width_t CountTrailingZeros64(std::uint64_t bit_string) {
    return bit_string == 0 ? 64U : __builtin_ctzll(bit_string);
}

inline bit_width_t CountTrailingZeros(std::uint32_t bit_string) {
    return bit_string == 0 ? 32U : __builtin_ctz(bit_string);
}

#elif defined(_MSC_VER)
// https://docs.microsoft.com/en-us/cpp/intrinsics/x64-amd64-intrinsics-list?view=vs-2019
inline bit_width_t CountLeadingZeros64(std::uint64_t bit_string) {
    unsigned long leading_zero = 0;
    return _BitScanReverse64(&leading_zero, bit_string) ? 63 - leading_zero : 64U;
}

inline bit_width_t CountLeadingZeros(std::uint32_t bit_string) {
    unsigned long leading_zero = 0;
    return _BitScanReverse(&leading_zero, bit_string) ? 31 - leading_zero : 32U;
}

inline bit_width_t CountTrailingZeros64(std::uint64_t bit_string) {
    unsigned long trailing_zero = 0;
    return _BitScanForward64(&trailing_zero, bit_string) ? trailing_zero : 64U;
}

inline bit_width_t CountTrailingZeros(std::uint32_t bit_string) {
    unsigned long trailing_zero = 0;
    return _BitScanForward(&trailing_zero, bit_string) ? trailing_zero : 32U;
}
#else
inline bit_width_t CountLeadingZeros(std::uint64_t bit_string) {
    if (bit_string == 0) {
        return 64;
    }
    bit_width_t n = 1;
    std::uint32_t x = (bit_string >> 32);
    if (x == 0) {
        n += 32;
        x = (int)bit_string;
    }
    if (x >> 16 == 0) {
        n += 16;
        x <<= 16;
    }
    if (x >> 24 == 0) {
        n += 8;
        x <<= 8;
    }
    if (x >> 28 == 0) {
        n += 4;
        x <<= 4;
    }
    if (x >> 30 == 0) {
        n += 2;
        x <<= 2;
    }
    n -= x >> 31;
    return n;
}

inline bit_width_t CountLeadingZeros(std::uint32_t bit_string) {
    if (bit_string == 0) {
        return 32;
    }
    bit_width_t n = 1;
    if (bit_string >> 16 == 0) {
        n += 16;
        bit_string <<= 16;
    }
    if (bit_string >> 24 == 0) {
        n += 8;
        bit_string <<= 8;
    }
    if (bit_string >> 28 == 0) {
        n += 4;
        bit_string <<= 4;
    }
    if (bit_string >> 30 == 0) {
        n += 2;
        bit_string <<= 2;
    }
    n -= bit_string >> 31;
    return n;
}

inline bit_width_t CountTrailingZeros(std::uint64_t bit_string) {
    if (bit_string == 0) {
        return 64;
    }
    uint32_t x = 0;
    uint32_t y = 0;
    uint16_t n = 63;
    y = (std::uint32_t)bit_string;
    if (y != 0) {
        n = n - 32;
        x = y;
    } else {
        x = (std::uint32_t)(bit_string >> 32);
    }
    y = x << 16;
    if (y != 0) {
        n = n - 16;
        x = y;
    }
    y = x << 8;
    if (y != 0) {
        n = n - 8;
        x = y;
    }
    y = x << 4;
    if (y != 0) {
        n = n - 4;
        x = y;
    }
    y = x << 2;
    if (y != 0) {
        n = n - 2;
        x = y;
    }
    return n - ((x << 1) >> 31);
}
#endif

}  // namespace improbable::phtree::detail

#endif  // PHTREE_COMMON_BITS_H
