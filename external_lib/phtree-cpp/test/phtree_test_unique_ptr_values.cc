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

namespace phtree_test_unique_ptr_values {

template <dimension_t DIM>
using TestPoint = PhPoint<DIM>;

template <dimension_t DIM, typename T>
using TestTree = PhTree<DIM, T>;

class IntRng {
  public:
    IntRng(int minIncl, int maxExcl) : eng(7), rnd{minIncl, maxExcl} {}

    int next() {
        return rnd(eng);
    }

  private:
    std::default_random_engine eng;
    std::uniform_int_distribution<int> rnd;
};

struct IdObj {
    IdObj() = default;

    explicit IdObj(const size_t i) : _i(static_cast<int>(i)){};

    bool operator==(const IdObj& rhs) const {
        return _i == rhs._i;
    }

    IdObj& operator=(IdObj const& rhs) = default;

    int _i;
};

using Id = std::unique_ptr<IdObj>;

struct PointDistance {
    PointDistance(double distance, size_t id) : _distance(distance), _id(id) {}

    double _distance;
    size_t _id;
};

bool comparePointDistance(PointDistance& i1, PointDistance& i2) {
    return (i1._distance < i2._distance);
}

template <dimension_t DIM>
double distance(const TestPoint<DIM>& p1, const TestPoint<DIM>& p2) {
    double sum2 = 0;
    for (dimension_t i = 0; i < DIM; i++) {
        double d = p1[i] - p2[i];
        sum2 += d * d;
    }
    return sqrt(sum2);
}

template <dimension_t DIM>
double distanceL1(const TestPoint<DIM>& p1, const TestPoint<DIM>& p2) {
    double sum = 0;
    for (dimension_t i = 0; i < DIM; i++) {
        sum += std::abs(p1[i] - p2[i]);
    }
    return sum;
}

template <dimension_t DIM>
void generateCube(std::vector<TestPoint<DIM>>& points, size_t N) {
    IntRng rng(-1000, 1000);
    auto refTree = std::map<TestPoint<DIM>, size_t>();

    points.reserve(N);
    for (size_t i = 0; i < N; i++) {
        auto point = TestPoint<DIM>{rng.next(), rng.next(), rng.next()};
        if (refTree.count(point) != 0) {
            i--;
            continue;
        }

        refTree.emplace(point, i);
        points.push_back(point);
    }
    ASSERT_EQ(refTree.size(), N);
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
        ASSERT_EQ(tree.count(p), 0u);
        ASSERT_EQ(tree.end(), tree.find(p));

        if (i % 2 == 0) {
            ASSERT_TRUE(tree.emplace(p, std::make_unique<IdObj>(i)).second);
        } else {
            Id id = std::make_unique<IdObj>(i);
            ASSERT_TRUE(tree.emplace(p, std::move(id)).second);
        }
        ASSERT_EQ(tree.count(p), 1u);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(i, (*tree.find(p))->_i);
        ASSERT_EQ(i + 1u, tree.size());

        // try adding it again
        ASSERT_FALSE(tree.emplace(p, std::make_unique<IdObj>(i)).second);
        ASSERT_EQ(tree.count(p), 1u);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(i, (*tree.find(p))->_i);
        ASSERT_EQ(i + 1u, tree.size());
        ASSERT_FALSE(tree.empty());
    }

    for (int i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        auto q = tree.begin_query({p, p});
        ASSERT_NE(q, tree.end());
        ASSERT_EQ(i, (*q)->_i);
        ++q;
        ASSERT_EQ(q, tree.end());
    }

    PhTreeDebugHelper::CheckConsistency(tree);

    for (int i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        ASSERT_NE(tree.find(p), tree.end());
        ASSERT_EQ(tree.count(p), 1u);
        ASSERT_EQ(i, (*tree.find(p))->_i);
        ASSERT_EQ(1u, tree.erase(p));

        ASSERT_EQ(tree.count(p), 0u);
        ASSERT_EQ(tree.end(), tree.find(p));
        ASSERT_EQ(N - i - 1u, tree.size());

        // try remove again
        ASSERT_EQ(0u, tree.erase(p));
        ASSERT_EQ(tree.count(p), 0u);
        ASSERT_EQ(tree.end(), tree.find(p));
        ASSERT_EQ(N - i - 1u, tree.size());
        if (i < N - 1) {
            ASSERT_FALSE(tree.empty());
        }
    }
    ASSERT_EQ(0u, tree.size());
    ASSERT_TRUE(tree.empty());
    PhTreeDebugHelper::CheckConsistency(tree);
}

TEST(PhTreeTestUniquePtr, SmokeTestBasicOps) {
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

TEST(PhTreeTestUniquePtr, TestUpdateWithRelocate) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::array<scalar_64_t, 4> deltas{0, 1, 10, 100};
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t d_n = 0;
    for (int x = 0; x < 10; ++x) {
        int i = 0;
        for (auto& p : points) {
            auto pOld = p;
            d_n = (d_n + 1) % deltas.size();
            scalar_64_t delta = deltas[d_n];
            TestPoint<dim> pNew{pOld[0] + delta, pOld[1] + delta, pOld[2] + delta};
            if (delta > 0 && tree.find(pNew) != tree.end()) {
                // Skip this, there is already another entry
                ASSERT_EQ(0, tree.relocate(pOld, pNew));
            } else {
                ASSERT_EQ(1, tree.relocate(pOld, pNew));
                if (delta > 0) {
                    // second time fails because value has already been moved
                    ASSERT_EQ(0, tree.relocate(pOld, pNew));
                }
                ASSERT_EQ(i, (*tree.find(pNew))->_i);
                p = pNew;
            }
            ++i;
        }
        PhTreeDebugHelper::CheckConsistency(tree);
    }

    ASSERT_EQ(N, tree.size());
    tree.clear();

    // Check that empty tree works
    ASSERT_EQ(0, tree.relocate(points[0], points[1]));
    // Check that small tree works
    tree.emplace(points[0], std::make_unique<IdObj>(1));
    ASSERT_EQ(1u, tree.relocate(points[0], points[1]));
    ASSERT_EQ(tree.end(), tree.find(points[0]));
    ASSERT_EQ(1, (*tree.find(points[1]))->_i);
    ASSERT_EQ(1u, tree.size());
    tree.clear();

    // check that existing destination fails
    tree.emplace(points[0], std::make_unique<IdObj>(1));
    tree.emplace(points[1], std::make_unique<IdObj>(2));
    ASSERT_EQ(0, tree.relocate(points[0], points[1]));
}

TEST(PhTreeTestUniquePtr, TestUpdateWithRelocateIf) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::array<scalar_64_t, 4> deltas{0, 1, 10, 100};
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t d_n = 0;
    for (int x = 0; x < 10; ++x) {
        int i = 0;
        size_t done = 0;
        auto pred = [](const Id& id) { return id->_i % 2 == 0; };
        for (auto& p : points) {
            auto pOld = p;
            d_n = (d_n + 1) % deltas.size();
            scalar_64_t delta = deltas[d_n];
            TestPoint<dim> pNew{pOld[0] + delta, pOld[1] + delta, pOld[2] + delta};
            if ((delta > 0 && tree.find(pNew) != tree.end()) || (i % 2 != 0)) {
                // Skip this, there is already another entry
                ASSERT_EQ(0, tree.relocate_if(pOld, pNew, pred));
            } else {
                ASSERT_EQ(1, tree.relocate_if(pOld, pNew, pred));
                if (delta > 0) {
                    // second time fails because value has already been moved
                    ASSERT_EQ(0, tree.relocate_if(pOld, pNew, pred));
                }
                ASSERT_EQ(i, (*tree.find(pNew))->_i);
                p = pNew;
                ++done;
            }
            ++i;
        }
        ASSERT_GT(done, i * 0.4);
        ASSERT_LT(done, i * 0.6);
        PhTreeDebugHelper::CheckConsistency(tree);
    }

    ASSERT_EQ(N, tree.size());
    tree.clear();

    // Check that empty tree works
    auto pred = [](const Id&) { return true; };
    ASSERT_EQ(0, tree.relocate_if(points[0], points[1], pred));
    // Check that small tree works
    tree.emplace(points[0], std::make_unique<IdObj>(1));
    ASSERT_EQ(1, tree.relocate_if(points[0], points[1], pred));
    ASSERT_EQ(tree.end(), tree.find(points[0]));
    ASSERT_EQ(1, (*tree.find(points[1]))->_i);
    ASSERT_EQ(1u, tree.size());
}

}  // namespace phtree_test_unique_ptr_values
