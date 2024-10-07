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

#include "phtree/common/common.h"
#include "phtree/converter.h"
#include <include/gtest/gtest.h>

using namespace improbable::phtree;
using namespace improbable::phtree::detail;

TEST(PhTreeCommonTest, NumberOfDivergingBits) {
    double d1 = -55;
    double d2 = 7;

    auto l1 = ScalarConverterIEEE::pre(d1);
    auto l2 = ScalarConverterIEEE::pre(d2);
    scalar_64_t l_min = std::numeric_limits<scalar_64_t>::lowest();
    scalar_64_t l_max = std::numeric_limits<scalar_64_t>::max();

    bit_width_t x = NumberOfDivergingBits(PhPoint<2>({l1, l1}), PhPoint<2>({l2, l2}));
    ASSERT_EQ(64u, x);
    x = NumberOfDivergingBits(PhPoint<2>({-1, -1}), PhPoint<2>({l_min, l_min}));
    ASSERT_EQ(63u, x);
    x = NumberOfDivergingBits(PhPoint<2>({1, 1}), PhPoint<2>({l_max, l_max}));
    ASSERT_EQ(63u, x);

    x = NumberOfDivergingBits(PhPoint<2>({l1, l2}), PhPoint<2>({l1, l2}));
    ASSERT_EQ(0u, x);

    // PhPointD{679.186, 519.897, 519.897}
    PhPoint<3> p1{0x4085397c9ffc65e8, 0x40803f2cf7158e9a, 0x40803f2cf7158e9a};
    // PhPointD{35.5375, 8.69049, 8.69049}
    PhPoint<3> p2{0x4041c4ce0e8a359e, 0x40216187a0776fd5, 0x40216187a0776fd5};
    x = NumberOfDivergingBits(p1, p2);
    ASSERT_EQ(56u, x);

    // PhPointD{132.406, 219.74, 219.74}
    PhPoint<3> p20{0x40608cffffe5b480, 0x406b77aff096adc1, 0x406b77aff096adc1};
    // PhPointD{679.186, 519.897, 519.897}
    PhPoint<3> p21{0x4085397c9ffc65e8, 0x40803f2cf7158e9a, 0x40803f2cf7158e9a};
    x = NumberOfDivergingBits(p20, p21);
    ASSERT_EQ(56u, x);
}
