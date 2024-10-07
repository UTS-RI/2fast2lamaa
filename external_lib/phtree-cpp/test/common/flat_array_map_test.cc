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

#include "phtree/common/flat_array_map.h"
#include <include/gtest/gtest.h>
#include <random>

using namespace improbable::phtree;

TEST(PhTreeFlatArrayMapTest, SmokeTest) {
    const int max_size = 8;

    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, max_size - 1);

    for (int i = 0; i < 10; i++) {
        detail::array_map<size_t, size_t, max_size> test_map;
        std::map<size_t, size_t> reference_map;
        for (int j = 0; j < 2 * max_size; j++) {
            size_t val = cube_distribution(random_engine);
            bool hasVal = test_map.find(val) != test_map.end();
            bool hasValRef = reference_map.find(val) != reference_map.end();
            ASSERT_EQ(hasVal, hasValRef);
            if (!hasVal) {
                reference_map.emplace(val, val);
                test_map.emplace(val, val);
            }
            ASSERT_EQ(test_map.size(), reference_map.size());
            for (auto it : reference_map) {
                size_t vRef = it.first;
                size_t vMap = test_map.find(vRef)->second;
                ASSERT_EQ(vMap, vRef);
            }
            for (auto it : test_map) {
                size_t v = it.first;
                size_t vRef = reference_map.find(v)->second;
                size_t vMap = test_map.find(v)->second;
                ASSERT_EQ(vMap, vRef);
            }
        }
    }
}

TEST(PhTreeFlatArrayMapTest, SmokeTestWithTryEmplace) {
    const int max_size = 8;

    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, max_size - 1);

    for (int i = 0; i < 10; i++) {
        detail::array_map<size_t, size_t, max_size> test_map;
        std::map<size_t, size_t> reference_map;
        for (int j = 0; j < 2 * max_size; j++) {
            size_t val = cube_distribution(random_engine);
            bool hasVal = test_map.find(val) != test_map.end();
            bool hasValRef = reference_map.find(val) != reference_map.end();
            ASSERT_EQ(hasVal, hasValRef);
            if (!hasVal) {
                reference_map.emplace(val, val);
                test_map.try_emplace(val, val);
            }
            ASSERT_EQ(test_map.size(), reference_map.size());
            for (auto it : reference_map) {
                size_t vRef = it.first;
                size_t vMap = test_map.find(vRef)->second;
                ASSERT_EQ(vMap, vRef);
            }
            for (auto it : test_map) {
                size_t v = it.first;
                size_t vRef = reference_map.find(v)->second;
                size_t vMap = test_map.find(v)->second;
                ASSERT_EQ(vMap, vRef);
            }
        }
    }
}

TEST(PhTreeFlatArrayMapTest, IteratorPostIncrementTest) {
    const int num_entries = 3;

    detail::array_map<size_t, size_t, 8> test_map;
    for (int j = 0; j < num_entries; j++) {
        size_t val = j * 2;
        bool hasVal = test_map.find(val) != test_map.end();
        if (!hasVal) {
            test_map.try_emplace(val, val);
        }
    }

    // test post increment
    auto it_post = test_map.begin();
    int n_post = 0;
    while (it_post != test_map.end()) {
        size_t v = (*it_post).first;
        auto it_find = test_map.find(v);
        ASSERT_EQ(it_post, it_find);
        // post increment
        auto it2 = it_post++;
        ASSERT_NE(it2, it_post);
        ++n_post;
    }
    ASSERT_EQ(num_entries, n_post);

    // test pre increment
    auto it_pre = test_map.begin();
    int n_pre = 0;
    while (it_pre != test_map.end()) {
        size_t v = (*it_pre).first;
        auto it_find = test_map.find(v);
        ASSERT_EQ(it_pre, it_find);
        auto it2 = ++it_pre;
        ASSERT_EQ(it2, it_pre);
        ++n_pre;
    }
    ASSERT_EQ(num_entries, n_pre);
}

TEST(PhTreeFlatArrayMapTest, EmptyTest) {
    detail::array_map<size_t, size_t, 8> test_map;
    ASSERT_EQ(test_map.size(), 0);
    ASSERT_EQ(test_map.begin(), test_map.end());
    ASSERT_EQ(test_map.find(0), test_map.end());
    ASSERT_EQ(test_map.lower_bound(0), test_map.end());
    ASSERT_EQ(test_map.lower_bound(7), test_map.end());
    ASSERT_FALSE(test_map.erase(0));
}

TEST(PhTreeFlatArrayMapTest, FailTest) {
    detail::array_map<size_t, size_t, 8> test_map;
    test_map.emplace(1);
    test_map.emplace(2);
    test_map.emplace(3);
    ASSERT_EQ(test_map.find(0), test_map.end());
    ASSERT_EQ(test_map.find(4), test_map.end());
    ASSERT_EQ(test_map.lower_bound(7), test_map.end());
    ASSERT_FALSE(test_map.erase(0));
    ASSERT_FALSE(test_map.erase(4));
}
