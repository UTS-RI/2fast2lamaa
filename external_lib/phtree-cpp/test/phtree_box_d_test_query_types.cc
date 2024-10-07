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

#include "phtree/phtree.h"
#include <include/gtest/gtest.h>
#include <random>

using namespace improbable::phtree;

namespace phtree_box_d_test_query_types {

template <dimension_t DIM>
using TestPoint = PhBoxD<DIM>;

template <dimension_t DIM, typename T>
using TestTree = PhTreeBoxD<DIM, T>;

TEST(PhTreeBoxDTestQueryTypes, SmokeTestQuery) {
    const dimension_t DIM = 2;
    TestTree<DIM, int> tree;

    PhPointD<DIM> p00{-10, -10};
    PhPointD<DIM> p11{10, 10};
    PhPointD<DIM> pm{0, 0};

    PhBoxD<DIM> b00{p00, pm};
    PhBoxD<DIM> b11{pm, p11};

    tree.emplace(b00, -1);
    tree.emplace(b11, 1);

    auto query_type = QueryInclude();

    // empty
    auto q1 = tree.begin_query({{-9, -9}, {9, 9}}, FilterNoOp(), query_type);
    ASSERT_EQ(q1, tree.end());

    // Find box00 but not box11
    auto q2 = tree.begin_query({{-11, -11}, {9, 9}}, FilterNoOp(), query_type);
    ASSERT_NE(q2, tree.end());
    ASSERT_EQ(-1, (*q2));
    ++q2;
    ASSERT_EQ(q2, tree.end());

    // Find box11 but not box00
    auto q3 = tree.begin_query({{-9, -9}, {11, 11}}, FilterNoOp(), query_type);
    ASSERT_NE(q3, tree.end());
    ASSERT_EQ(1, (*q3));
    ++q3;
    ASSERT_EQ(q3, tree.end());
}

}  // namespace phtree_box_d_test_query_types
