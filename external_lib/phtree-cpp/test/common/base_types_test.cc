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

#include "phtree/common/base_types.h"
#include <include/gtest/gtest.h>
#include <unordered_set>

using namespace improbable::phtree;

TEST(PhTreeBaseTypesTest, PhPointD) {
    PhPointD<3> point{1, 2, 3};

    for (int i = 0; i < 3; i++) {
        ASSERT_EQ(point[i], i + 1);
    }

    // try assigning coordinates
    point = {7, 8, 9};
    for (int i = 0; i < 3; i++) {
        ASSERT_EQ(point[i], i + 7);
    }

    // Test '<<' operator
    std::stringstream ss;
    ss << point;
    ASSERT_EQ("[7,8,9]", ss.str());
}

TEST(PhTreeBaseTypesTest, PhPointF) {
    PhPoint<3, float> point{1, 2, 3};

    for (int i = 0; i < 3; i++) {
        ASSERT_EQ(point[i], i + 1);
    }

    // try assigning coordinates
    point = {7, 8, 9};
    for (int i = 0; i < 3; i++) {
        ASSERT_EQ(point[i], i + 7);
    }

    // Test '<<' operator
    std::stringstream ss;
    ss << point;
    ASSERT_EQ("[7,8,9]", ss.str());
}

TEST(PhTreeBaseTypesTest, PhBoxD) {
    PhBoxD<3> box({1, 2, 3}, {4, 5, 6});

    for (int i = 0; i < 3; i++) {
        ASSERT_EQ(box.min()[i], i + 1);
        ASSERT_EQ(box.max()[i], i + 4);
    }

    // try assigning coordinates
    box.min() = {7, 8, 9};
    box.max() = {10, 11, 12};
    for (int i = 0; i < 3; i++) {
        ASSERT_EQ(box.min()[i], i + 7);
        ASSERT_EQ(box.max()[i], i + 10);
    }

    // Test that hash function works
    std::unordered_set<PhBoxD<3>> map;
    ASSERT_TRUE(map.insert(box).second);
    ASSERT_NE(42, std::hash<PhBoxD<3>>()(box));

    // Test '<<' operator
    std::stringstream ss;
    ss << box;
    ASSERT_EQ("[7,8,9]:[10,11,12]", ss.str());
}
