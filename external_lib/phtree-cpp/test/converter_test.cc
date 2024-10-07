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

#include "phtree/converter.h"
#include <include/gtest/gtest.h>

using namespace improbable::phtree;

template <typename PRE, typename POST>
void test_less_than(PRE pre, POST post, float d1, float d2) {
    auto l1 = pre(d1);
    auto l2 = pre(d2);
    ASSERT_LT(l1, l2);
    ASSERT_EQ(d1, post(l1));
    ASSERT_EQ(d2, post(l2));
}

template <typename PRE, typename POST>
void testAll(PRE pre, POST post) {
    test_less_than(pre, post, 55.0f, 71.0f);
    test_less_than(pre, post, -55.0f, 7.0f);
    test_less_than(pre, post, -55.0f, -7.0f);
}

TEST(PhTreePreprocessorTest, IEEE_Double_SmokeTest) {
    auto pre = [](double d) { return ScalarConverterIEEE::pre(d); };
    auto post = [](scalar_64_t s) { return ScalarConverterIEEE::post(s); };
    testAll(pre, post);
}

TEST(PhTreePreprocessorTest, IEEE_Float_SmokeTest) {
    auto pre = [](float f) { return ScalarConverterIEEE::pre(f); };
    auto post = [](scalar_32_t s) { return ScalarConverterIEEE::post(s); };
    testAll(pre, post);
}
