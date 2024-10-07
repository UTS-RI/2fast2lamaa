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

#include "phtree/filter.h"
#include <include/gtest/gtest.h>
#include <random>

using namespace improbable::phtree;

TEST(PhTreeFilterTest, FilterSphereTest) {
    ConverterNoOp<2, scalar_64_t> conv{};
    FilterSphere filter{{5, 3}, 5, conv, DistanceEuclidean<2>{}};
    // root is always valid
    ASSERT_TRUE(filter.IsNodeValid({0, 0}, 63));
    // valid because node encompasses the circle
    ASSERT_TRUE(filter.IsNodeValid({1, 1}, 10));
    // valid because circle encompasses the node
    ASSERT_TRUE(filter.IsNodeValid({5, 5}, 2));
    // valid because circle encompasses the node AABB
    ASSERT_TRUE(filter.IsNodeValid({7, 7}, 1));
    // valid because circle touches the edge of the node AABB
    ASSERT_TRUE(filter.IsNodeValid({5, 9}, 1));
    // valid because circle cuts edge of node AABB
    ASSERT_TRUE(filter.IsNodeValid({12, 7}, 3));
    ASSERT_TRUE(filter.IsNodeValid({10, 7}, 2));
    // invalid because node is just outside the circle
    ASSERT_FALSE(filter.IsNodeValid({5, 10}, 1));
    ASSERT_FALSE(filter.IsNodeValid({12, 12}, 3));

    ASSERT_TRUE(filter.IsEntryValid({3, 7}, nullptr));
    ASSERT_TRUE(filter.IsEntryValid({5, 8}, nullptr));
    ASSERT_FALSE(filter.IsEntryValid({3, 8}, nullptr));
}

TEST(PhTreeFilterTest, FilterAABBTest) {
    ConverterNoOp<2, scalar_64_t> conv{};
    FilterAABB filter{{3, 3}, {7, 7}, conv};
    // root is always valid
    ASSERT_TRUE(filter.IsNodeValid({0, 0}, 63));
    // valid because node encompasses the AABB
    ASSERT_TRUE(filter.IsNodeValid({1, 1}, 10));
    // valid
    ASSERT_TRUE(filter.IsNodeValid({7, 7}, 1));
    // invalid
    ASSERT_FALSE(filter.IsNodeValid({88, 5}, 1));

    ASSERT_TRUE(filter.IsEntryValid({3, 7}, nullptr));
    ASSERT_FALSE(filter.IsEntryValid({2, 8}, nullptr));
}

TEST(PhTreeFilterTest, FilterNoOpSmokeTest) {
    auto filter = FilterNoOp();
    ASSERT_TRUE(filter.IsNodeValid<PhPoint<3>>({3, 7, 2}, 10));
    ASSERT_TRUE(filter.IsEntryValid<PhPoint<3>>({3, 7, 2}, 10));
}

template <typename FILTER_FN>
void TestAssignability() {
    ASSERT_TRUE(std::is_copy_constructible_v<FILTER_FN>);
    ASSERT_TRUE(std::is_copy_assignable_v<FILTER_FN>);
    ASSERT_TRUE(std::is_move_constructible_v<FILTER_FN>);
    ASSERT_TRUE(std::is_move_assignable_v<FILTER_FN>);
}

TEST(PhTreeFilterTest, FilterAssignableTest) {
    using CONV = ConverterIEEE<3>;
    using DIST = DistanceEuclidean<3>;
    TestAssignability<FilterNoOp>();
    TestAssignability<FilterAABB<CONV>>();
    TestAssignability<FilterSphere<CONV, DIST>>();
    TestAssignability<FilterMultiMapAABB<CONV>>();
    TestAssignability<FilterMultiMapSphere<CONV, DIST>>();
}

TEST(PhTreeFilterTest, ConverterAssignableTest) {
    TestAssignability<ConverterIEEE<3>>();
    TestAssignability<ScalarConverterIEEE>();
}

class TestConverter : public ConverterMultiply<2, 1, 1> {
  public:
    TestConverter() = default;

    TestConverter(const TestConverter&) = delete;
    TestConverter(TestConverter&&) = delete;
    TestConverter& operator=(const TestConverter&) = delete;
    TestConverter& operator=(TestConverter&&) = delete;
};

TEST(PhTreeFilterTest, ConstructFilterAABBTest) {
    TestConverter conv;
    FilterAABB filter1{{3, 3}, {7, 7}, conv};
    ASSERT_TRUE(filter1.IsNodeValid({0, 0}, 63));

    FilterAABB filter2{{3, 3}, {7, 7}, TestConverter()};
    ASSERT_TRUE(filter2.IsNodeValid({0, 0}, 63));
}

TEST(PhTreeFilterTest, ConstructFilterSphereTest) {
    DistanceL1<2> dist;
    TestConverter conv;
    FilterSphere filter1a{{3, 3}, 7, conv};
    ASSERT_TRUE(filter1a.IsNodeValid({0, 0}, 63));
    FilterSphere filter1b{{3, 3}, 7, conv, {}};
    ASSERT_TRUE(filter1b.IsNodeValid({0, 0}, 63));
    FilterSphere filter1c{{3, 3}, 7, conv, dist};
    ASSERT_TRUE(filter1c.IsNodeValid({0, 0}, 63));
    FilterSphere filter1d{{3, 3}, 7, conv, DistanceL1<2>{}};
    ASSERT_TRUE(filter1d.IsNodeValid({0, 0}, 63));

    FilterSphere filter2{{3, 3}, 7, TestConverter()};
    ASSERT_TRUE(filter2.IsNodeValid({0, 0}, 63));
}
