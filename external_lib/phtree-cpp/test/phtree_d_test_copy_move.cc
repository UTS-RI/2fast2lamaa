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

namespace phtree_d_test_copy_move {

template <dimension_t DIM>
using TestPoint = PhPointD<DIM>;

template <dimension_t DIM, typename T>
using TestTree = PhTreeD<DIM, T>;

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

struct IdCopyOnly {
    explicit IdCopyOnly(const size_t i) : _i{i} {}

    IdCopyOnly() = default;
    IdCopyOnly(const IdCopyOnly& other) = default;
    IdCopyOnly(IdCopyOnly&& other) = delete;
    // IdCopyOnly& operator=(const IdCopyOnly& other) = default;
    IdCopyOnly& operator=(const IdCopyOnly& other) {
        _i = other._i;
        return *this;
    }
    IdCopyOnly& operator=(IdCopyOnly&& other) = delete;
    ~IdCopyOnly() = default;

    bool operator==(const IdCopyOnly& rhs) const {
        return _i == rhs._i;
    }

    size_t _i{};
};

struct IdMoveOnly {
    explicit IdMoveOnly(const size_t i) : _i{i} {}

    IdMoveOnly() = default;
    IdMoveOnly(const IdMoveOnly& other) = delete;
    IdMoveOnly(IdMoveOnly&& other) = default;
    IdMoveOnly& operator=(const IdMoveOnly& other) = delete;
    IdMoveOnly& operator=(IdMoveOnly&& other) = default;
    ~IdMoveOnly() = default;

    bool operator==(const IdMoveOnly& rhs) const {
        return _i == rhs._i;
    }

    size_t _i{};
};

// Assert that copy-ctr is not called even when available
struct IdCopyOrMove {
    explicit IdCopyOrMove(const size_t i) : _i{i} {}

    IdCopyOrMove() = default;
    IdCopyOrMove(const IdCopyOrMove&) {
        assert(false);
    }
    IdCopyOrMove(IdCopyOrMove&& other) = default;
    IdCopyOrMove& operator=(const IdCopyOrMove&) {
        assert(false);
    }
    IdCopyOrMove& operator=(IdCopyOrMove&& other) = default;
    ~IdCopyOrMove() = default;

    bool operator==(const IdCopyOrMove& rhs) const {
        return _i == rhs._i;
    }

    size_t _i{};
};

template <dimension_t DIM>
void generateCube(std::vector<TestPoint<DIM>>& points, size_t N) {
    DoubleRng rng(-1000, 1000);
    auto refTree = std::map<TestPoint<DIM>, size_t>();

    points.reserve(N);
    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM> point{};
        for (dimension_t d = 0; d < DIM; ++d) {
            point[d] = rng.next();
        }
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

template <dimension_t DIM, typename Id>
void SmokeTestBasicOps_QueryAndErase(TestTree<DIM, Id>& tree, std::vector<TestPoint<DIM>>& points) {
    size_t N = points.size();

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        auto q = tree.begin_query({p, p});
        ASSERT_NE(q, tree.end());
        ASSERT_EQ(i, (*q)._i);
        ++q;
        ASSERT_EQ(q, tree.end());
    }

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        auto q = tree.begin_knn_query(1, p, DistanceEuclidean<DIM>());
        ASSERT_NE(q, tree.end());
        ASSERT_EQ(i, (*q)._i);
        ++q;
        ASSERT_EQ(q, tree.end());
    }

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        TestPoint<DIM> pOld = p;
        for (dimension_t d = 0; d < DIM; ++d) {
            p[d] += 10000;
        }
        auto r = tree.relocate(pOld, p);
        ASSERT_EQ(r, 1u);
    }

    PhTreeDebugHelper::CheckConsistency(tree);

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        ASSERT_NE(tree.find(p), tree.end());
        ASSERT_EQ(tree.count(p), 1u);
        ASSERT_EQ(i, tree.find(p)->_i);
        if (i % 2 == 0) {
            ASSERT_EQ(1u, tree.erase(p));
        } else {
            auto iter = tree.find(p);
            ASSERT_EQ(1u, tree.erase(iter));
        }

        ASSERT_EQ(tree.count(p), 0u);
        ASSERT_EQ(tree.end(), tree.find(p));
        ASSERT_EQ(N - i - 1, tree.size());

        // try remove again
        ASSERT_EQ(0u, tree.erase(p));
        ASSERT_EQ(tree.count(p), 0u);
        ASSERT_EQ(tree.end(), tree.find(p));
        ASSERT_EQ(N - i - 1, tree.size());
        if (i < N - 1) {
            ASSERT_FALSE(tree.empty());
        }
    }
    ASSERT_EQ(0u, tree.size());
    ASSERT_TRUE(tree.empty());
    PhTreeDebugHelper::CheckConsistency(tree);
}

template <dimension_t DIM, typename Id>
void SmokeTestBasicOps(size_t N) {
    TestTree<DIM, Id> tree;
    std::vector<TestPoint<DIM>> points;
    generateCube(points, N);

    ASSERT_EQ(0u, tree.size());
    ASSERT_TRUE(tree.empty());
    PhTreeDebugHelper::CheckConsistency(tree);

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        ASSERT_EQ(tree.count(p), 0u);
        ASSERT_EQ(tree.end(), tree.find(p));

        Id id(i);
        if (i % 4 == 0) {
            ASSERT_TRUE(tree.try_emplace(p, id).second);
        } else if (i % 4 == 1) {
            ASSERT_TRUE(tree.emplace(p, id).second);
        } else if (i % 4 == 2) {
            tree[p] = id;
        } else {
            ASSERT_TRUE(tree.insert(p, id).second);
        }
        ASSERT_EQ(tree.count(p), 1u);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(id._i, tree.find(p)->_i);
        ASSERT_EQ(i + 1, tree.size());

        // try adding it again
        ASSERT_FALSE(tree.insert(p, id).second);
        ASSERT_FALSE(tree.emplace(p, id).second);
        ASSERT_EQ(tree.count(p), 1u);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(id._i, tree.find(p)->_i);
        ASSERT_EQ(i + 1, tree.size());
        ASSERT_FALSE(tree.empty());
    }

    SmokeTestBasicOps_QueryAndErase(tree, points);
}

TEST(PhTreeDTestCopyMove, SmokeTestBasicOpsCopyOnly) {
    SmokeTestBasicOps<1, IdCopyOnly>(100);
    SmokeTestBasicOps<3, IdCopyOnly>(100);
    SmokeTestBasicOps<6, IdCopyOnly>(100);
    SmokeTestBasicOps<10, IdCopyOnly>(100);
    SmokeTestBasicOps<20, IdCopyOnly>(100);
    SmokeTestBasicOps<63, IdCopyOnly>(100);
}

template <dimension_t DIM, typename Id>
void SmokeTestBasicOpsMoveOnly(size_t N) {
    TestTree<DIM, Id> tree;
    std::vector<TestPoint<DIM>> points;
    generateCube(points, N);

    ASSERT_EQ(0u, tree.size());
    ASSERT_TRUE(tree.empty());
    PhTreeDebugHelper::CheckConsistency(tree);

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        ASSERT_EQ(tree.count(p), 0u);
        ASSERT_EQ(tree.end(), tree.find(p));

        if (i % 2 == 0) {
            ASSERT_TRUE(tree.try_emplace(p, Id(i)).second);
        } else if (i % 4 == 1) {
            tree[p] = Id(i);
        } else {
            ASSERT_TRUE(tree.emplace(p, Id(i)).second);
        }
        ASSERT_EQ(tree.count(p), 1u);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(i, tree.find(p)->_i);
        ASSERT_EQ(i + 1, tree.size());

        // try adding it again
        ASSERT_FALSE(tree.try_emplace(p, Id(i)).second);
        ASSERT_FALSE(tree.emplace(p, Id(i)).second);
        ASSERT_EQ(tree.count(p), 1u);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(i, tree.find(p)->_i);
        ASSERT_EQ(i + 1, tree.size());
        ASSERT_FALSE(tree.empty());
    }

    SmokeTestBasicOps_QueryAndErase(tree, points);
}

TEST(PhTreeDTestCopyMove, SmokeTestBasicOpsMoveOnly) {
    SmokeTestBasicOpsMoveOnly<1, IdMoveOnly>(100);
    SmokeTestBasicOpsMoveOnly<3, IdMoveOnly>(100);
    SmokeTestBasicOpsMoveOnly<6, IdMoveOnly>(100);
    SmokeTestBasicOpsMoveOnly<10, IdMoveOnly>(100);
    SmokeTestBasicOpsMoveOnly<20, IdMoveOnly>(100);
    SmokeTestBasicOpsMoveOnly<63, IdMoveOnly>(100);
}

TEST(PhTreeDTestCopyMove, SmokeTestBasicOpsCopyFails) {
    SmokeTestBasicOpsMoveOnly<1, IdCopyOrMove>(100);
    SmokeTestBasicOpsMoveOnly<3, IdCopyOrMove>(100);
    SmokeTestBasicOpsMoveOnly<6, IdCopyOrMove>(100);
    SmokeTestBasicOpsMoveOnly<10, IdCopyOrMove>(100);
    SmokeTestBasicOpsMoveOnly<20, IdCopyOrMove>(100);
    SmokeTestBasicOpsMoveOnly<63, IdCopyOrMove>(100);
}

}  // namespace phtree_d_test_copy_move
