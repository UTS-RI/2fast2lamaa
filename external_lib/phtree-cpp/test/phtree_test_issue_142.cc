/*
 * Copyright 2023 Tilmann Zäschke
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

// Test that the CALLBACK in Windows SDK does not conflict with the PhTree
#define CALLBACK
#include "phtree/phtree.h"
#include "phtree/phtree_multimap.h"
#include <include/gtest/gtest.h>

using namespace improbable::phtree;

namespace phtree_test_issue_142 {

TEST(PhTreeTestIssue142, TestIssue142) {
    // The real test is ABOVE, see "#define CALLBACK"
    auto tree = PhTreeD<2, int>();
    PhPointD<2> p{1, 2};
    tree.emplace(p, 42);
}

TEST(PhTreeTestIssue142, TestIssue142_MM) {
    // The real test is ABOVE, see "#define CALLBACK"
    auto tree = PhTreeMultiMapD<2, int>();
    PhPointD<2> p{1, 2};
    tree.emplace(p, 42);
}

}  // namespace phtree_test_issue_142
