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
#include <unordered_map>

using namespace improbable::phtree;

namespace phtree_box_f_test {

template <dimension_t DIM>
using TestPoint = PhBoxF<DIM>;

template <dimension_t DIM, typename T>
using TestTree = PhTreeBoxF<DIM, T>;

class FloatRng {
  public:
    FloatRng(double minIncl, double maxExcl)
    : eng(), rnd{static_cast<float>(minIncl), static_cast<float>(maxExcl)} {}

    float next() {
        return rnd(eng);
    }

  private:
    std::default_random_engine eng;
    std::uniform_real_distribution<float> rnd;
};

struct Id {
    Id() = default;

    explicit Id(const size_t i) : _i(i){};

    bool operator==(const Id& rhs) const {
        return _i == rhs._i;
    }

    size_t _i;
};

struct PointDistance {
    PointDistance(double distance, size_t id) : _distance(distance), _id(id) {}

    double _distance;
    size_t _id;
};

bool comparePointDistance(PointDistance& i1, PointDistance& i2) {
    return (i1._distance < i2._distance);
}

template <dimension_t DIM>
double distance(const PhPointD<DIM>& p1, const PhPointD<DIM>& p2) {
    double sum2 = 0;
    for (dimension_t i = 0; i < DIM; i++) {
        double d = p1[i] - p2[i];
        sum2 += d * d;
    }
    return sqrt(sum2);
}

template <dimension_t DIM>
void generateCube(std::vector<TestPoint<DIM>>& points, size_t N, float boxLen = 10) {
    FloatRng rng(-1000, 1000);
    auto refTree = std::unordered_map<TestPoint<DIM>, size_t>();

    points.reserve(N);
    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM> box{};
        for (dimension_t d = 0; d < DIM; ++d) {
            box.min()[d] = rng.next();
            box.max()[d] = box.min()[d] + boxLen;
        }
        if (refTree.count(box) != 0) {
            i--;
            continue;
        }

        refTree.emplace(box, i + 1);
        points.push_back(box);
    }
    ASSERT_EQ(refTree.size(), N);
    ASSERT_EQ(points.size(), N);
}

template <dimension_t DIM>
void SmokeTestBasicOps(size_t N) {
    TestTree<DIM, Id> tree;
    std::vector<TestPoint<DIM>> points;
    generateCube(points, N);

    ASSERT_EQ(0, tree.size());
    ASSERT_TRUE(tree.empty());
    PhTreeDebugHelper::CheckConsistency(tree);

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        ASSERT_EQ(tree.count(p), 0);
        ASSERT_EQ(tree.end(), tree.find(p));

        Id id(i);
        if (i % 2 == 0) {
            ASSERT_TRUE(tree.emplace(p, id).second);
        } else {
            ASSERT_TRUE(tree.insert(p, id).second);
        }
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(id._i, tree.find(p)->_i);
        ASSERT_EQ(i + 1, tree.size());

        // try add again
        ASSERT_FALSE(tree.insert(p, id).second);
        ASSERT_FALSE(tree.emplace(p, id).second);
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(id._i, tree.find(p)->_i);
        ASSERT_EQ(i + 1, tree.size());
        ASSERT_FALSE(tree.empty());
    }

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        // With intersection queries we may get multiple results.
        size_t found = 0;
        for (auto q = tree.begin_query(p); q != tree.end(); ++q) {
            found += (i == (*q)._i);
        }
        ASSERT_EQ(1, found);
    }

    PhTreeDebugHelper::CheckConsistency(tree);

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        ASSERT_NE(tree.find(p), tree.end());
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_EQ(i, tree.find(p)->_i);
        ASSERT_EQ(1, tree.erase(p));

        ASSERT_EQ(tree.count(p), 0);
        ASSERT_EQ(tree.end(), tree.find(p));
        ASSERT_EQ(N - i - 1, tree.size());

        // try remove again
        ASSERT_EQ(0, tree.erase(p));
        ASSERT_EQ(tree.count(p), 0);
        ASSERT_EQ(tree.end(), tree.find(p));
        ASSERT_EQ(N - i - 1, tree.size());
        if (i < N - 1) {
            ASSERT_FALSE(tree.empty());
        }
    }
    ASSERT_EQ(0, tree.size());
    ASSERT_TRUE(tree.empty());
    PhTreeDebugHelper::CheckConsistency(tree);
}

TEST(PhTreeBoxFTest, SmokeTestBasicOps) {
    SmokeTestBasicOps<1>(100);
    SmokeTestBasicOps<3>(10000);
    SmokeTestBasicOps<6>(10000);
    SmokeTestBasicOps<10>(1000);
    SmokeTestBasicOps<20>(1000);
    SmokeTestBasicOps<31>(100);
}

TEST(PhTreeBoxFTest, TestDebug) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 1000;

    std::vector<TestPoint<dim>> points;
    generateCube(points, N);

    using Debug = PhTreeDebugHelper;
    ASSERT_LE(10, Debug::ToString(tree, Debug::PrintDetail::name).length());
    ASSERT_GE(10, Debug::ToString(tree, Debug::PrintDetail::entries).length());
    ASSERT_GE(100, Debug::ToString(tree, Debug::PrintDetail::tree).length());
    ASSERT_EQ(0, Debug::GetStats(tree).size_);
    Debug::CheckConsistency(tree);

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        Id id(i);
        ASSERT_TRUE(tree.insert(p, id).second);
    }

    ASSERT_LE(10, Debug::ToString(tree, Debug::PrintDetail::name).length());
    ASSERT_LE(N * 10, Debug::ToString(tree, Debug::PrintDetail::entries).length());
    ASSERT_LE(N * 10, Debug::ToString(tree, Debug::PrintDetail::tree).length());
    ASSERT_EQ(N, Debug::GetStats(tree).size_);
    Debug::CheckConsistency(tree);

    tree.clear();

    ASSERT_LE(10, Debug::ToString(tree, Debug::PrintDetail::name).length());
    ASSERT_GE(10, Debug::ToString(tree, Debug::PrintDetail::entries).length());
    ASSERT_GE(100, Debug::ToString(tree, Debug::PrintDetail::tree).length());
    ASSERT_EQ(0, Debug::GetStats(tree).size_);
    Debug::CheckConsistency(tree);
}

TEST(PhTreeBoxFTest, TestInsert) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 1000;

    std::vector<TestPoint<dim>> points;
    generateCube(points, N);

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        Id id(i);
        ASSERT_EQ(true, tree.insert(p, id).second);
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_EQ(id._i, tree.find(p)->_i);

        // try add again
        ASSERT_EQ(false, tree.insert(p, id).second);
        ASSERT_EQ(i, tree.insert(p, id).first._i);
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_EQ(id._i, tree.find(p)->_i);
    }
    ASSERT_EQ(N, tree.size());

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        // With intersection queries we may get multiple results.
        size_t found = 0;
        for (auto q = tree.begin_query(p); q != tree.end(); ++q) {
            found += (i == (*q)._i);
        }
        ASSERT_EQ(1, found);
    }

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_EQ(i, tree.find(p)->_i);
    }
}

TEST(PhTreeBoxFTest, TestEmplace) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 1000;

    std::vector<TestPoint<dim>> points;
    generateCube(points, N);

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        Id id(i);
        ASSERT_EQ(true, tree.emplace(p, id).second);
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_EQ(id._i, tree.find(p)->_i);
        ASSERT_EQ(i + 1, tree.size());

        // try add again, this should _not_ replace the existing value
        Id id2(i + N);
        ASSERT_EQ(false, tree.emplace(p, id2).second);
        ASSERT_EQ(i, tree.emplace(p, id).first._i);
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_EQ(id._i, tree.find(p)->_i);

        // Check that the returned value is a reference
        tree.emplace(p, id2).first._i++;
        ASSERT_EQ(i + 1, tree.emplace(p, id).first._i);
        tree.emplace(p, id2).first = id;
        ASSERT_EQ(i, tree.emplace(p, id).first._i);
    }
    ASSERT_EQ(N, tree.size());

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        // With intersection queries we may get multiple results.
        size_t found = 0;
        for (auto q = tree.begin_query(p); q != tree.end(); ++q) {
            found += (i == (*q)._i);
        }
        ASSERT_EQ(1, found);
    }

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_EQ(i, tree.find(p)->_i);
    }
}

TEST(PhTreeBoxFTest, TestSquareBrackets) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 1000;

    std::vector<TestPoint<dim>> points;
    generateCube(points, N);

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        Id id(i);
        ASSERT_EQ(0, tree[p]._i);
        ASSERT_EQ(tree.count(p), 1);
        if (i % 2 == 0) {
            tree[p]._i = i;
        } else {
            tree[p] = id;
        }
        ASSERT_EQ(id._i, tree.find(p)->_i);
        ASSERT_EQ(i + 1, tree.size());

        // try `add` again
        ASSERT_EQ(i, tree[p]._i);
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_EQ(id._i, tree.find(p)->_i);
    }
    ASSERT_EQ(N, tree.size());

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        // With intersection queries we may get multiple results.
        size_t found = 0;
        for (auto q = tree.begin_query(p); q != tree.end(); ++q) {
            found += (i == (*q)._i);
        }
        ASSERT_EQ(1, found);
    }

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_EQ(i, tree.find(p)->_i);
        ASSERT_EQ(i, tree[p]._i);
    }
}

template <dimension_t DIM>
void populate(
    PhTreeBoxF<DIM, size_t>& tree,
    std::vector<TestPoint<DIM>>& points,
    size_t N,
    double boxLen = 10) {
    generateCube(points, N, boxLen);
    for (size_t i = 0; i < N; i++) {
        ASSERT_TRUE(tree.insert(points[i], i + 1).second);
    }
    ASSERT_EQ(N, tree.size());
}

template <dimension_t DIM>
void populate(
    TestTree<DIM, Id>& tree, std::vector<TestPoint<DIM>>& points, size_t N, double boxLen = 10) {
    generateCube(points, N, (float)boxLen);
    for (size_t i = 0; i < N; i++) {
        ASSERT_TRUE(tree.emplace(points[i], i + 1).second);
    }
    ASSERT_EQ(N, tree.size());
}

TEST(PhTreeBoxFTest, TestClear) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 100;
    std::vector<TestPoint<dim>> points;

    ASSERT_TRUE(tree.empty());
    tree.clear();
    ASSERT_TRUE(tree.empty());

    populate(tree, points, N);

    ASSERT_FALSE(tree.empty());
    tree.clear();
    ASSERT_TRUE(tree.empty());
    points.clear();

    // try again
    populate(tree, points, N);

    ASSERT_FALSE(tree.empty());
    tree.clear();
    ASSERT_TRUE(tree.empty());
    points.clear();
}

TEST(PhTreeBoxFTest, TestFind) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t i = 0;
    for (auto& p : points) {
        // test commutativity
        ASSERT_NE(tree.find(p), tree.end());
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(tree.find(p)->_i, i + 1);
        i++;
    }

    TestPoint<dim> p({1, 1, 1}, {2, 2, 10000000});
    auto result = tree.find(p);
    ASSERT_EQ(result, tree.end());
    ASSERT_EQ(tree.end(), result);
}

TEST(PhTreeBoxFTest, TestUpdateWithEmplace) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    float delta = 20;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    for (auto& p : points) {
        auto pOld = p;
        TestPoint<dim> pNew(
            {pOld.min()[0] + delta, pOld.min()[1] + delta, pOld.min()[2] + delta},
            {pOld.max()[0] + delta, pOld.max()[1] + delta, pOld.max()[2] + delta});
        size_t n = tree.erase(pOld);
        ASSERT_EQ(1u, n);
        tree.emplace(pNew, 42u);
        ASSERT_EQ(1, tree.count(pNew));
        ASSERT_EQ(0, tree.count(pOld));
        p = pNew;
    }

    ASSERT_EQ(N, tree.size());
    tree.clear();
}

TEST(PhTreeBoxFTest, TestUpdateWithEmplaceHint) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::array<float, 4> deltas{0.f, 0.1f, 1.f, 10.f};
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t d_n = 0;
    for (auto& p : points) {
        auto pOld = p;
        d_n = (d_n + 1) % deltas.size();
        float delta = deltas[d_n];
        PhPointF<dim> min{pOld.min()[0] + delta, pOld.min()[1] + delta, pOld.min()[2] + delta};
        PhPointF<dim> max{pOld.max()[0] + delta, pOld.max()[1] + delta, pOld.max()[2] + delta};
        TestPoint<dim> pNew{min, max};
        auto iter = tree.find(pOld);
        size_t n = tree.erase(iter);
        ASSERT_EQ(1u, n);
        tree.emplace_hint(iter, pNew, 42u);
        ASSERT_EQ(1, tree.count(pNew));
        if (delta != 0.0) {
            ASSERT_EQ(0, tree.count(pOld));
        }
        p = pNew;
    }

    ASSERT_EQ(N, tree.size());
    tree.clear();
}

TEST(PhTreeBoxFTest, TestEraseByIterator) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t i = 0;
    for (auto& p : points) {
        auto iter = tree.find(p);
        ASSERT_NE(tree.end(), iter);
        size_t count = tree.erase(iter);
        ASSERT_EQ(1u, count);
        ASSERT_EQ(tree.end(), tree.find(p));
        i++;
    }

    ASSERT_EQ(0, tree.erase(tree.end()));
}

TEST(PhTreeBoxFTest, TestEraseByIteratorQuery) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    for (size_t i = 0; i < N; ++i) {
        auto iter = tree.begin();
        ASSERT_NE(tree.end(), iter);
        size_t count = tree.erase(iter);
        ASSERT_EQ(1u, count);
    }

    ASSERT_EQ(0, tree.erase(tree.end()));
}

TEST(PhTreeBoxFTest, TestExtent) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    int num_e = 0;
    auto qE = tree.begin();
    while (qE != tree.end()) {
        ASSERT_TRUE(qE->_i >= 1);
        qE++;
        num_e++;
    }
    ASSERT_EQ(N, num_e);
}

TEST(PhTreeBoxFTest, TestRangeBasedForLoop) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    int num_e1 = 0;
    for (auto& x : tree) {
        ASSERT_TRUE(x._i >= 1);
        num_e1++;
    }
    ASSERT_EQ(N, num_e1);

    size_t num_e2 = 0;
    for (auto& x : tree) {
        ASSERT_TRUE(x._i >= 1);
        num_e2++;
    }
    ASSERT_EQ(N, num_e2);
}

template <dimension_t DIM>
void referenceQuery(
    std::vector<TestPoint<DIM>>& points,
    PhPointF<DIM>& min,
    PhPointF<DIM>& max,
    std::set<size_t>& result) {
    for (size_t i = 0; i < points.size(); i++) {
        auto& p = points[i];
        auto pMin = p.min();
        auto pMax = p.max();
        bool match = true;
        for (dimension_t d = 0; d < DIM; d++) {
            match &= pMax[d] >= min[d] && pMin[d] <= max[d];
        }
        if (match) {
            result.insert(i + 1);
        }
    }
}

// We use 'int&' because gtest does not compile with assertions in non-void functions.
template <dimension_t DIM>
void testQuery(PhPointF<DIM>& min, PhPointF<DIM>& max, size_t N, int& result) {
    TestTree<DIM, Id> tree;
    std::vector<TestPoint<DIM>> points;
    populate(tree, points, N);

    std::set<size_t> referenceResult;
    referenceQuery(points, min, max, referenceResult);

    result = 0;
    for (auto it = tree.begin_query({min, max}); it != tree.end(); it++) {
        auto& x = *it;
        ASSERT_GE(x._i, 0);
        ASSERT_EQ(referenceResult.count(x._i), 1);
        result++;
    }
    ASSERT_EQ(referenceResult.size(), result);
}

TEST(PhTreeBoxFTest, TestWindowQuery0) {
    const dimension_t dim = 3;
    PhPointF<dim> p{-10000, -10000, -10000};
    int n = 0;
    testQuery<dim>(p, p, 10000, n);
    ASSERT_EQ(0, n);
}

TEST(PhTreeBoxFTest, TestWindowQuery1) {
    size_t N = 1000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t n = 0;
    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        // With intersection queries we may get multiple results.
        size_t found = 0;
        for (auto q = tree.begin_query(p); q != tree.end(); ++q) {
            found += (i + 1 == (*q)._i);
        }
        ASSERT_EQ(1, found);
        n++;
    }
    ASSERT_EQ(N, n);
}

TEST(PhTreeBoxFTest, TestWindowQueryMany) {
    const dimension_t dim = 3;
    PhPointF<dim> min{-100, -100, -100};
    PhPointF<dim> max{100, 100, 100};
    int n = 0;
    testQuery<dim>(min, max, 10000, n);
    ASSERT_LE(3, n);
    ASSERT_GE(100, n);
}

TEST(PhTreeBoxFTest, TestWindowQueryAll) {
    const dimension_t dim = 3;
    const size_t N = 10000;
    PhPointF<dim> min{-10000, -10000, -10000};
    PhPointF<dim> max{10000, 10000, 10000};
    int n = 0;
    testQuery<dim>(min, max, N, n);
    ASSERT_EQ(N, n);
}

TEST(PhTreeBoxFTest, TestWindowQueryManyMoving) {
    size_t N = 10000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    float query_length = 200;
    size_t nn = 0;
    for (int i = -120; i < 120; i++) {
        PhPointF<dim> min{i * 10.0f, i * 9.0f, i * 11.0f};
        PhPointF<dim> max{i * 10 + query_length, i * 9 + query_length, i * 11 + query_length};
        std::set<size_t> referenceResult;
        referenceQuery(points, min, max, referenceResult);

        size_t n = 0;
        for (auto it = tree.begin_query({min, max}); it != tree.end(); it++) {
            auto& x = *it;
            ASSERT_EQ(referenceResult.count(x._i), 1);
            n++;
            nn++;
        }
        ASSERT_EQ(referenceResult.size(), n);

        // basic check to ensure healthy queries
        if (i > -50 && i < 50) {
            ASSERT_LE(1, n);
        }
        ASSERT_GE(100, n);
    }
    ASSERT_LE(500, nn);
    ASSERT_GE(5000, nn);
}

TEST(PhTreeBoxFTest, TestWindowQueryManyMovingPoint) {
    size_t N = 10000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N, 100);

    size_t nTotal = 0;
    for (int i = -120; i < 120; i++) {
        PhPointF<dim> min_max{i * 10.0f, i * 9.0f, i * 11.0f};
        std::set<size_t> referenceResult;
        referenceQuery(points, min_max, min_max, referenceResult);

        int n = 0;
        for (auto it = tree.begin_query({min_max, min_max}); it != tree.end(); it++) {
            auto& x = *it;
            ASSERT_EQ(referenceResult.count(x._i), 1);
            n++;
        }
        ASSERT_EQ(referenceResult.size(), n);
        nTotal += n;

        // basic check to ensure healthy queries
        ASSERT_GE(N / 10, n);
    }
    ASSERT_LE(10, nTotal);
}

TEST(PhTreeBoxFTest, TestWindowForEachManyMovingPoint) {
    size_t N = 10000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N, 100);

    size_t nTotal = 0;
    for (int i = -120; i < 120; i++) {
        PhPointF<dim> min_max{i * 10.0f, i * 9.0f, i * 11.0f};
        std::set<size_t> referenceResult;
        referenceQuery(points, min_max, min_max, referenceResult);

        struct Counter {
            void operator()(TestPoint<dim>, Id& t) {
                ++n_;
                ASSERT_EQ(referenceResult.count(t._i), 1);
            }
            std::set<size_t>& referenceResult;
            size_t n_ = 0;
        };

        size_t n = 0;
        Counter callback{referenceResult, 0};
        tree.for_each({min_max, min_max}, callback);
        n += callback.n_;
        ASSERT_EQ(referenceResult.size(), n);
        nTotal += n;

        // basic check to ensure healthy queries
        ASSERT_GE(N / 10, n);
    }
    ASSERT_LE(10, nTotal);
}

TEST(PhTreeBoxFTest, SmokeTestPointAPI) {
    TestPoint<3> p({1, 2, 3}, {4, 5, 6});
    (void)p;
}

TEST(PhTreeBoxFTest, SmokeTestTreeAPI) {
    std::map<int, Id*> mapPtr;
    PhTreeBoxF<3, Id*> treePtr;
    Id* idPtr = new Id(1);
    treePtr.emplace(TestPoint<3>({1, 2, 3}, {4, 5, 6}), idPtr);
    treePtr.clear();
    delete idPtr;

    std::map<int, const Id> mapConst;
    PhTreeBoxF<3, const Id> treeConst;
    treeConst.emplace(TestPoint<3>({1, 2, 3}, {4, 5, 6}), Id(1));
}

}  // namespace phtree_box_f_test
