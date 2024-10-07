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

#include "phtree/phtree_multimap.h"
#include <include/gtest/gtest.h>
#include <random>
#include <unordered_map>

using namespace improbable::phtree;

namespace phtree_multimap_d_test_unique_ptr_values {

// Number of entries that have the same coordinate
static const size_t NUM_DUPL = 4;
static const double WORLD_MIN = -1000;
static const double WORLD_MAX = 1000;

template <dimension_t DIM>
using TestPoint = PhPointD<DIM>;

class DoubleRng {
  public:
    DoubleRng(double minIncl, double maxExcl) : eng(), rnd{minIncl, maxExcl} {}

    double next() {
        return rnd(eng);
    }

  private:
    std::default_random_engine eng;
    std::uniform_real_distribution<double> rnd;
};

struct IdObj {
    IdObj() = default;

    explicit IdObj(const int i) : _i(i), data_{0} {};
    explicit IdObj(const size_t i) : _i(static_cast<int>(i)), data_{0} {};

    bool operator==(const IdObj& rhs) const noexcept {
        return _i == rhs._i;
    }

    int _i;
    int data_;
};

using Id = std::unique_ptr<IdObj>;
}  // namespace phtree_multimap_d_test_unique_ptr_values

namespace std {
template <>
struct hash<phtree_multimap_d_test_unique_ptr_values::Id> {
    size_t operator()(const phtree_multimap_d_test_unique_ptr_values::Id& x) const {
        return std::hash<int>{}(x->_i);
    }
};
};  // namespace std
struct equal_to_content {
    bool operator()(
        const phtree_multimap_d_test_unique_ptr_values::Id& x1,
        const phtree_multimap_d_test_unique_ptr_values::Id& x2) const {
        return (*x1) == (*x2);
    }
};
struct less_content {
    bool operator()(
        const phtree_multimap_d_test_unique_ptr_values::Id& x1,
        const phtree_multimap_d_test_unique_ptr_values::Id& x2) const {
        return (*x1)._i < (*x2)._i;
    }
};

namespace phtree_multimap_d_test_unique_ptr_values {

template <dimension_t DIM, typename T>
using TestTree = PhTreeMultiMap<
    DIM,
    T,
    ConverterIEEE<DIM>,
    b_plus_tree_hash_set<T, std::hash<Id>, equal_to_content>>;
// using TestTree = PhTreeMultiMap<DIM, T, ConverterIEEE<DIM>, std::unordered_set<T, std::hash<Id>,
// equal_to_content>>; using TestTree = PhTreeMultiMap<DIM, T, ConverterIEEE<DIM>, std::set<T,
// less_content>>;

template <dimension_t DIM>
void generateCube(std::vector<TestPoint<DIM>>& points, size_t N) {
    assert(N % NUM_DUPL == 0);
    DoubleRng rng(WORLD_MIN, WORLD_MAX);
    auto reference_set = std::unordered_map<TestPoint<DIM>, size_t>();

    points.reserve(N);
    for (size_t i = 0; i < N / NUM_DUPL; i++) {
        // create duplicates, i.e. entries with the same coordinates. However, avoid unintentional
        // duplicates.
        TestPoint<DIM> key{};
        for (dimension_t d = 0; d < DIM; ++d) {
            key[d] = rng.next();
        }
        if (reference_set.count(key) != 0) {
            i--;
            continue;
        }
        reference_set.emplace(key, i);
        for (size_t dupl = 0; dupl < NUM_DUPL; dupl++) {
            auto point = TestPoint<DIM>(key);
            points.emplace_back(point);
        }
    }
    ASSERT_EQ(reference_set.size(), N / NUM_DUPL);
    ASSERT_EQ(points.size(), N);
}

template <dimension_t DIM>
void SmokeTestBasicOps(int N) {
    TestTree<DIM, Id> tree;
    std::vector<TestPoint<DIM>> points;
    generateCube(points, N);

    ASSERT_EQ(0u, tree.size());
    ASSERT_TRUE(tree.empty());
    PhTreeDebugHelper::CheckConsistency(tree);

    for (int i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        ASSERT_LE(tree.count(p), i % NUM_DUPL);
        if (i % NUM_DUPL == 0) {
            ASSERT_EQ(tree.end(), tree.find(p));
        }

        Id id2(new IdObj{i});
        //        Id id3(new IdObj{i});
        //        ASSERT_EQ(id2.get(), id3.get());
        //        ASSERT_TRUE(id2 == id3);
        //        ASSERT_EQ(id2, id3);
        if (i % 4 == 0) {
            ASSERT_TRUE(tree.emplace(p, std::make_unique<IdObj>(i)).second);
        } else if (i % 4 == 1) {
            ASSERT_TRUE(tree.emplace(p, new IdObj{i}).second);
        } else if (i % 4 == 2) {
            ASSERT_TRUE(tree.try_emplace(p, new IdObj{i}).second);
        } else {
            Id id = std::make_unique<IdObj>(i);
            ASSERT_TRUE(tree.emplace(p, std::move(id)).second);
        }
        Id id = std::make_unique<IdObj>(i);
        ASSERT_EQ(tree.count(p), i % NUM_DUPL + 1);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(i, (*tree.find(p, id))->_i);
        ASSERT_EQ(i + 1u, tree.size());

        // try adding it again
        ASSERT_FALSE(tree.try_emplace(p, std::make_unique<IdObj>(i)).second);
        ASSERT_FALSE(tree.emplace(p, std::make_unique<IdObj>(i)).second);
        ASSERT_EQ(tree.count(p), i % NUM_DUPL + 1);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(i, (*tree.find(p, std::make_unique<IdObj>(i)))->_i);
        ASSERT_EQ(i + 1u, tree.size());
        ASSERT_FALSE(tree.empty());
    }

    for (int i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        auto q = tree.begin_query({p, p});
        ASSERT_NE(q, tree.end());
        for (size_t j = 0; j < NUM_DUPL; j++) {
            ASSERT_EQ(i / NUM_DUPL, (*q)->_i / NUM_DUPL);
            ++q;
        }
        ASSERT_EQ(q, tree.end());
    }

    PhTreeDebugHelper::CheckConsistency(tree);

    for (int i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        ASSERT_NE(tree.find(p), tree.end());
        size_t expected_remaining = (N - i - 1) % NUM_DUPL + 1;
        ASSERT_EQ(tree.count(p), expected_remaining);
        ASSERT_EQ(i, (*tree.find(p, std::make_unique<IdObj>(i)))->_i);
        if (i % 3 == 0) {
            ASSERT_EQ(1u, tree.erase(p, std::make_unique<IdObj>(i)));
        } else {
            auto iter = tree.find(p, std::make_unique<IdObj>(i));
            ASSERT_EQ(1u, tree.erase(iter));
        }

        ASSERT_EQ(tree.count(p), expected_remaining - 1);
        if (expected_remaining - 1 == 0) {
            ASSERT_EQ(tree.end(), tree.find(p));
        }
        ASSERT_EQ(N - i - 1u, tree.size());

        // try remove again
        ASSERT_EQ(0u, tree.erase(p, std::make_unique<IdObj>(i)));
        ASSERT_EQ(tree.count(p), expected_remaining - 1);
        if (expected_remaining - 1 == 0) {
            ASSERT_EQ(tree.end(), tree.find(p));
        }
        ASSERT_EQ(N - i - 1u, tree.size());
        if (i < N - 1) {
            ASSERT_FALSE(tree.empty());
        }
    }
    ASSERT_EQ(0u, tree.size());
    ASSERT_TRUE(tree.empty());
    PhTreeDebugHelper::CheckConsistency(tree);
}

TEST(PhTreeMMDTestUniquePtr, SmokeTestBasicOps) {
    SmokeTestBasicOps<1>(10000);
    SmokeTestBasicOps<3>(10000);
    SmokeTestBasicOps<6>(10000);
    SmokeTestBasicOps<10>(1000);
    SmokeTestBasicOps<20>(100);
}

template <dimension_t DIM>
void populate(TestTree<DIM, Id>& tree, std::vector<TestPoint<DIM>>& points, size_t N) {
    generateCube(points, N);
    for (size_t i = 0; i < N; i++) {
        ASSERT_TRUE(tree.emplace(points[i], std::make_unique<IdObj>(i)).second);
    }
    ASSERT_EQ(N, tree.size());
}

TEST(PhTreeMMDTestUniquePtr, TestUpdateWithRelocate) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 1000;
    std::array<double, 4> deltas{0, 1, 10, 100};
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    for (auto delta : deltas) {
        int i = 0;
        for (auto& p : points) {
            auto pOld = p;
            TestPoint<dim> pNew{pOld[0] + delta, pOld[1] + delta, pOld[2] + delta};
            ASSERT_EQ(1u, tree.relocate(pOld, pNew, std::make_unique<IdObj>(i)));
            if (delta > 0) {
                // second time fails because value has already been moved
                ASSERT_EQ(0u, tree.relocate(pOld, pNew, std::make_unique<IdObj>(i)));
            }
            ASSERT_EQ(i, (*tree.find(pNew, std::make_unique<IdObj>(i)))->_i);
            p = pNew;
            ++i;
        }
        PhTreeDebugHelper::CheckConsistency(tree);
    }

    ASSERT_EQ(N, tree.size());
    tree.clear();
}

TEST(PhTreeMMDTestUniquePtr, TestUpdateWithRelocateCornerCases) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    TestPoint<dim> point0{1, 2, 3};
    TestPoint<dim> point1{4, 5, 6};

    // Check that empty tree works
    ASSERT_EQ(0u, tree.relocate(point0, point1, std::make_unique<IdObj>(42)));

    // Check that small tree works
    tree.emplace(point0, std::make_unique<IdObj>(1));
    ASSERT_EQ(1u, tree.relocate(point0, point1, std::make_unique<IdObj>(1)));
    ASSERT_EQ(tree.end(), tree.find(point0, std::make_unique<IdObj>(1)));
    ASSERT_EQ(1, (*tree.find(point1, std::make_unique<IdObj>(1)))->_i);
    ASSERT_EQ(1u, tree.size());
    PhTreeDebugHelper::CheckConsistency(tree);
    tree.clear();

    // check that existing destination fails
    tree.emplace(point0, std::make_unique<IdObj>(1));
    tree.emplace(point1, std::make_unique<IdObj>(1));
    ASSERT_EQ(0u, tree.relocate(point0, point1, std::make_unique<IdObj>(1)));
    PhTreeDebugHelper::CheckConsistency(tree);
    tree.clear();

    // check that missing source bucket fails
    tree.emplace(point1, std::make_unique<IdObj>(1));
    ASSERT_EQ(0u, tree.relocate(point0, point1, std::make_unique<IdObj>(0)));
    PhTreeDebugHelper::CheckConsistency(tree);
    tree.clear();

    // check that missing source value fails (target bucket exists)
    tree.emplace(point0, std::make_unique<IdObj>(0));
    tree.emplace(point1, std::make_unique<IdObj>(1));
    ASSERT_EQ(0u, tree.relocate(point0, point1, std::make_unique<IdObj>(2)));
    PhTreeDebugHelper::CheckConsistency(tree);
    tree.clear();

    // check that missing source value fails (target bucket missing)
    tree.emplace(point0, std::make_unique<IdObj>(0));
    ASSERT_EQ(0u, tree.relocate(point0, point1, std::make_unique<IdObj>(2)));
    PhTreeDebugHelper::CheckConsistency(tree);
}

TEST(PhTreeMMDTestUniquePtr, TestUpdateWithRelocateIf) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::array<double, 4> deltas{0, 1, 10, 100};
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    for (auto delta : deltas) {
        size_t done = 0;
        for (int i = 0; size_t(i) < N; ++i) {
            auto pred = [&i](const Id& id) { return id->_i == i; };
            auto pOld = points[i];
            TestPoint<dim> pNew{pOld[0] + delta, pOld[1] + delta, pOld[2] + delta};
            ASSERT_EQ(1u, tree.relocate_if(pOld, pNew, pred));
            if (delta > 0) {
                // second time fails because value has already been moved
                ASSERT_EQ(0u, tree.relocate_if(pOld, pNew, pred));
            }
            ASSERT_EQ(i, (*tree.find(pNew, std::make_unique<IdObj>(i)))->_i);
            ++done;
            points[i] = pNew;
        }
        ASSERT_EQ(done, N);
        PhTreeDebugHelper::CheckConsistency(tree);
    }
    ASSERT_EQ(N, tree.size());
    tree.clear();
}

TEST(PhTreeMMDTestUniquePtr, TestUpdateWithRelocateIfCornerCases) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    TestPoint<dim> point0{1, 2, 3};
    TestPoint<dim> point1{4, 5, 6};
    auto TRUE = [](const Id&) { return true; };
    auto TWO = [](const Id& id) { return id->_i == 2; };

    // Check that empty tree works
    ASSERT_EQ(0u, tree.relocate_if(point0, point1, TRUE));

    // Check that small tree works
    tree.emplace(point0, std::make_unique<IdObj>(1));
    ASSERT_EQ(1u, tree.relocate_if(point0, point1, TRUE));
    ASSERT_EQ(tree.end(), tree.find(point0));
    ASSERT_EQ(1, (*tree.find(point1))->_i);
    ASSERT_EQ(1u, tree.size());
    PhTreeDebugHelper::CheckConsistency(tree);
    tree.clear();

    // check that existing destination fails
    tree.emplace(point0, std::make_unique<IdObj>(1));
    tree.emplace(point1, std::make_unique<IdObj>(1));
    ASSERT_EQ(0u, tree.relocate_if(point0, point1, TRUE));
    PhTreeDebugHelper::CheckConsistency(tree);
    tree.clear();

    // check that missing source bucket fails
    tree.emplace(point1, std::make_unique<IdObj>(1));
    ASSERT_EQ(0u, tree.relocate_if(point0, point1, TRUE));
    PhTreeDebugHelper::CheckConsistency(tree);
    tree.clear();

    // check that missing source value fails (target bucket exists)
    tree.emplace(point0, std::make_unique<IdObj>(0));
    tree.emplace(point1, std::make_unique<IdObj>(1));
    ASSERT_EQ(0u, tree.relocate_if(point0, point1, TWO));
    PhTreeDebugHelper::CheckConsistency(tree);
    tree.clear();

    // check that missing source value fails (target bucket missing)
    tree.emplace(point0, std::make_unique<IdObj>(0));
    ASSERT_EQ(0u, tree.relocate_if(point0, point1, TWO));
    PhTreeDebugHelper::CheckConsistency(tree);
}

}  // namespace phtree_multimap_d_test_unique_ptr_values
