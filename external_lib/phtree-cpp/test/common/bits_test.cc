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

#include "phtree/common/bits.h"
#include <include/gtest/gtest.h>
#include <random>

using namespace improbable::phtree;
using namespace detail;

TEST(PhTreeBitsTest, CountLeadingZeros64) {
    std::uint64_t x = 1;
    x <<= 63;
    for (int i = 0; i < 64; i++) {
        int ctz = CountLeadingZeros64(x);
        ASSERT_EQ(i, ctz);
        x >>= 1;
    }
}

TEST(PhTreeBitsTest, CountTrailingZeros64) {
    std::uint64_t x = 1;
    for (int i = 0; i < 64; i++) {
        int ctz = CountTrailingZeros64(x);
        ASSERT_EQ(i, ctz);
        x <<= 1;
    }
}

TEST(PhTreeBitsTest, CountLeadingZeros32) {
    std::uint32_t x = 1;
    x <<= 31;
    for (int i = 0; i < 32; i++) {
        int ctz = CountLeadingZeros(x);
        ASSERT_EQ(i, ctz);
        x >>= 1;
    }
}

TEST(PhTreeBitsTest, CountTrailingZeros32) {
    std::uint32_t x = 1;
    for (int i = 0; i < 32; i++) {
        int ctz = CountTrailingZeros(x);
        ASSERT_EQ(i, ctz);
        x <<= 1;
    }
}
