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

#include "phtree/distance.h"
#include <include/gtest/gtest.h>
#include <random>

using namespace improbable::phtree;

// NOTE: These are very superficial tests. Proper testing is done in the respective PhTree tests.

TEST(PhTreeDistanceTest, DoubleEuclidean) {
    auto distance = DistanceEuclidean<2>();
    ASSERT_DOUBLE_EQ(5, distance(PhPointD<2>{-1, -1}, PhPointD<2>{2, 3}));
}

TEST(PhTreeDistanceTest, DoubleL1) {
    auto distance = DistanceL1<2>();
    ASSERT_DOUBLE_EQ(7, distance(PhPointD<2>{-1, -1}, PhPointD<2>{2, 3}));
}

TEST(PhTreeDistanceTest, DoubleChebyshev) {
    auto distance = DistanceChebyshev<2>();
    ASSERT_DOUBLE_EQ(4, distance(PhPointD<2>{-1, -1}, PhPointD<2>{2, 3}));
}

TEST(PhTreeDistanceTest, FloatEuclidean) {
    auto distance = DistanceEuclidean<2>();
    ASSERT_DOUBLE_EQ(5, distance(PhPointF<2>{-1, -1}, PhPointF<2>{2, 3}));
}

TEST(PhTreeDistanceTest, FloatL1) {
    auto distance = DistanceL1<2>();
    ASSERT_DOUBLE_EQ(7, distance(PhPointF<2>{-1, -1}, PhPointF<2>{2, 3}));
}

TEST(PhTreeDistanceTest, FloatChebyshev) {
    auto distance = DistanceChebyshev<2>();
    ASSERT_DOUBLE_EQ(4, distance(PhPointF<2>{-1, -1}, PhPointF<2>{2, 3}));
}

TEST(PhTreeDistanceTest, LongEuclidean) {
    auto distance = DistanceEuclidean<2>();
    ASSERT_DOUBLE_EQ(5, distance(PhPoint<2>{-1, -1}, PhPoint<2>{2, 3}));
}

TEST(PhTreeDistanceTest, LongL1) {
    auto distance = DistanceL1<2>();
    ASSERT_DOUBLE_EQ(7, distance(PhPoint<2>{-1, -1}, PhPoint<2>{2, 3}));
}

TEST(PhTreeDistanceTest, LongChebyshev) {
    auto distance = DistanceChebyshev<2>();
    ASSERT_DOUBLE_EQ(4, distance(PhPoint<2>{-1, -1}, PhPoint<2>{2, 3}));
}

