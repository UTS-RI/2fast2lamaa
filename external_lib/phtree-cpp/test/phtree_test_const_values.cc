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

namespace phtree_test_const_values {

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

struct Id {
    Id() = default;

    explicit Id(const size_t i) : _i{static_cast<int>(i)} {}

    bool operator==(const Id& rhs) const {
        return _i == rhs._i;
    }

    int _i;
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
double distance(const TestPoint<DIM>& p1, const TestPoint<DIM>& p2) {
    double sum2 = 0;
    for (dimension_t i = 0; i < DIM; i++) {
        double d = (double)p1[i] - (double)p2[i];
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
void SmokeTestBasicOps() {
    TestTree<DIM, const Id> tree;
    size_t N = 10000;

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
        auto q = tree.begin_query({p, p});
        ASSERT_NE(q, tree.end());
        ASSERT_EQ(i, (*q)._i);
        ++q;
        ASSERT_EQ(q, tree.end());
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

TEST(PhTreeTestConst, SmokeTestBasicOps) {
    SmokeTestBasicOps<3>();
    SmokeTestBasicOps<6>();
    SmokeTestBasicOps<10>();
    SmokeTestBasicOps<20>();
}

TEST(PhTreeTestConst, TestDebug) {
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
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

TEST(PhTreeTestConst, TestInsert) {
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
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
        auto q = tree.begin_query({p, p});
        ASSERT_NE(q, tree.end());
        ASSERT_EQ(i, (*q)._i);
        ++q;
        ASSERT_EQ(q, tree.end());
    }

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_EQ(i, tree.find(p)->_i);
    }
}

TEST(PhTreeTestConst, TestEmplace) {
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
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
    }
    ASSERT_EQ(N, tree.size());

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        auto q = tree.begin_query({p, p});
        ASSERT_NE(q, tree.end());
        ASSERT_EQ(i, (*q)._i);
        ++q;
        ASSERT_EQ(q, tree.end());
    }

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_EQ(i, tree.find(p)->_i);
    }
}

TEST(PhTreeTestConst, TestSquareBrackets) {
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
    size_t N = 1000;

    std::vector<TestPoint<dim>> points;
    generateCube(points, N);

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        Id id(i);
        ASSERT_EQ(0, tree[p]._i);
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_EQ(i + 1, tree.size());
    }
    ASSERT_EQ(N, tree.size());

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        auto q = tree.begin_query({p, p});
        ASSERT_NE(q, tree.end());
        ASSERT_EQ(0, (*q)._i);
        ++q;
        ASSERT_EQ(q, tree.end());
    }
}

template <dimension_t DIM>
void populate(TestTree<DIM, const size_t>& tree, std::vector<TestPoint<DIM>>& points, size_t N) {
    generateCube(points, N);
    for (size_t i = 0; i < N; i++) {
        ASSERT_TRUE(tree.insert(points[i], i).second);
    }
    ASSERT_EQ(N, tree.size());
}

template <dimension_t DIM>
void populate(TestTree<DIM, const Id>& tree, std::vector<TestPoint<DIM>>& points, size_t N) {
    generateCube(points, N);
    for (size_t i = 0; i < N; i++) {
        ASSERT_TRUE(tree.emplace(points[i], (int)i).second);
    }
    ASSERT_EQ(N, tree.size());
}

TEST(PhTreeTestConst, TestClear) {
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
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

TEST(PhTreeTestConst, TestFind) {
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t i = 0;
    for (auto& p : points) {
        // test commutativity
        ASSERT_NE(tree.find(p), tree.end());
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(tree.find(p)->_i, i);
        i++;
    }

    TestPoint<dim> p{1, 1, 10000000};
    auto result = tree.find(p);
    ASSERT_EQ(result, tree.end());
    ASSERT_EQ(tree.end(), result);

    auto iter1 = tree.find(points[0]);
    auto iter2 = tree.find(points[0]);
    ASSERT_EQ(iter1, iter2);
    ASSERT_NE(tree.end(), iter1);
}

TEST(PhTreeTestConst, TestUpdateWithEmplace) {
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
    size_t N = 10000;
    int delta = 20;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    for (auto& p : points) {
        auto pOld = p;
        TestPoint<dim> pNew{pOld[0] + delta, pOld[1] + delta, pOld[2] + delta};
        size_t n = tree.erase(pOld);
        ASSERT_EQ(1u, n);
        tree.emplace(pNew, 42);
        ASSERT_EQ(1, tree.count(pNew));
        ASSERT_EQ(0, tree.count(pOld));
        p = pNew;
    }

    ASSERT_EQ(N, tree.size());
    tree.clear();
}

TEST(PhTreeTestConst, TestExtent) {
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    int num_e = 0;
    auto qE = tree.begin();
    while (qE != tree.end()) {
        ASSERT_TRUE(qE->_i > -1);
        qE++;
        num_e++;
    }
    ASSERT_EQ(N, num_e);

    auto iter1 = tree.begin();
    auto iter2 = tree.begin();
    ASSERT_EQ(iter1, iter2);
    ASSERT_NE(tree.end(), iter1);
}

template <dimension_t DIM, typename T>
struct FilterEvenId {
    [[nodiscard]] constexpr bool IsEntryValid(const PhPoint<DIM>&, const T& value) const {
        return value._i % 2 == 0;
    }
    [[nodiscard]] constexpr bool IsNodeValid(const PhPoint<DIM>&, int) const {
        return true;
    }
};

TEST(PhTreeTestConst, TestExtentFilter) {
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    int num_e = 0;
    auto qE = tree.begin(FilterEvenId<dim, Id>());
    while (qE != tree.end()) {
        ASSERT_TRUE(qE->_i > -1);
        ASSERT_TRUE(qE->_i % 2 == 0);
        qE++;
        num_e++;
    }
    ASSERT_EQ(N, num_e * 2);
}

TEST(PhTreeTestConst, TestRangeBasedForLoop) {
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t num_e1 = 0;
    for (auto& x : tree) {
        ASSERT_TRUE(x._i > -1);
        num_e1++;
    }
    ASSERT_EQ(N, num_e1);

    size_t num_e2 = 0;
    for (auto& x : tree) {
        ASSERT_TRUE(x._i > -1);
        num_e2++;
    }
    ASSERT_EQ(N, num_e2);
}

template <dimension_t DIM>
void referenceQuery(
    std::vector<TestPoint<DIM>>& points,
    TestPoint<DIM>& min,
    TestPoint<DIM>& max,
    std::set<size_t>& result) {
    for (size_t i = 0; i < points.size(); i++) {
        auto& p = points[i];
        bool match = true;
        for (dimension_t d = 0; d < DIM; d++) {
            match &= p[d] >= min[d] && p[d] <= max[d];
        }
        if (match) {
            result.insert(i);
        }
    }
}

// We use 'int&' because gtest does not compile with assertions in non-void functions.
template <dimension_t DIM>
void testQuery(TestPoint<DIM>& min, TestPoint<DIM>& max, size_t N, int& result) {
    TestTree<DIM, const Id> tree;
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

TEST(PhTreeTestConst, TestWindowQuery0) {
    const dimension_t dim = 3;
    TestPoint<dim> p{-10000, -10000, -10000};
    int n = 0;
    testQuery<dim>(p, p, 10000, n);
    ASSERT_EQ(0, n);
}

TEST(PhTreeTestConst, TestWindowQuery1) {
    size_t N = 1000;
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    int n = 0;
    for (size_t i = 0; i < N; i++) {
        PhPoint<dim>& p = points.at(i);
        auto q = tree.begin_query({p, p});
        ASSERT_NE(q, tree.end());
        // just read the entry
        auto& x = *q;
        ASSERT_EQ(i, x._i);
        ++q;
        ASSERT_EQ(q, tree.end());
        n++;
    }
    ASSERT_EQ(N, n);
}

TEST(PhTreeTestConst, TestWindowQueryMany) {
    const dimension_t dim = 3;
    TestPoint<dim> min{-100, -100, -100};
    TestPoint<dim> max{100, 100, 100};
    int n = 0;
    testQuery<dim>(min, max, 10000, n);
    ASSERT_LE(3, n);
    ASSERT_GE(100, n);
}

TEST(PhTreeTestConst, TestWindowQueryAll) {
    const dimension_t dim = 3;
    const size_t N = 10000;
    TestPoint<dim> min{-10000, -10000, -10000};
    TestPoint<dim> max{10000, 10000, 10000};
    int n = 0;
    testQuery<dim>(min, max, N, n);
    ASSERT_EQ(N, n);
}

TEST(PhTreeTestConst, TestWindowQueryManyMoving) {
    size_t N = 10000;
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    int query_length = 200;
    size_t nn = 0;
    for (std::int64_t i = -120; i < 120; i++) {
        TestPoint<dim> min{i * 10, i * 9, i * 11};
        TestPoint<dim> max{i * 10 + query_length, i * 9 + query_length, i * 11 + query_length};
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

TEST(PhTreeTestConst, TestWindowQueryIterators) {
    size_t N = 1000;
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    int n = 0;
    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        auto q1 = tree.begin_query({p, p});
        auto q2 = tree.begin_query({p, p});
        ASSERT_NE(q1, tree.end());
        ASSERT_NE(q2, tree.end());
        ASSERT_EQ(q1, q2);
        ++q1;
        ASSERT_NE(q1, q2);
        ++q2;
        n++;
    }
    ASSERT_EQ(N, n);
}

TEST(PhTreeTestConst, TestWindowQueryFilter) {
    const dimension_t dim = 3;
    TestTree<dim, const Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    int num_e = 0;
    TestPoint<dim> min{-100, -100, -100};
    TestPoint<dim> max{100, 100, 100};
    auto qE = tree.begin_query({min, max}, FilterEvenId<dim, Id>());
    while (qE != tree.end()) {
        ASSERT_TRUE(qE->_i > -1);
        ASSERT_TRUE(qE->_i % 2 == 0);
        qE++;
        num_e++;
    }
    ASSERT_LE(2, num_e);
    ASSERT_GE(50, num_e);
}

TEST(PhTreeTestConst, TestKnnQuery) {
    // deliberately allowing outside of main points range
    IntRng rng(-1500, 1500);
    const dimension_t dim = 3;
    const size_t N = 1000;
    const size_t Nq = 10;

    TestTree<dim, const Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    for (size_t round = 0; round < 100; round++) {
        TestPoint<dim> center{rng.next(), rng.next(), rng.next()};

        // sort points manually
        std::vector<PointDistance> sorted_data;
        for (size_t i = 0; i < points.size(); i++) {
            double dist = distance(center, points[i]);
            sorted_data.emplace_back(dist, i);
        }
        std::sort(sorted_data.begin(), sorted_data.end(), comparePointDistance);

        size_t n = 0;
        double prevDist = -1;
        auto q = tree.begin_knn_query(Nq, center, DistanceEuclidean<3>());
        while (q != tree.end()) {
            // just read the entry
            auto& e = *q;
            ASSERT_EQ(sorted_data[n]._distance, q.distance());
            ASSERT_EQ(sorted_data[n]._id, e._i);
            ASSERT_EQ(points[sorted_data[n]._id], q.first());
            ASSERT_EQ(sorted_data[n]._id, q.second()._i);
            ASSERT_GE(q.distance(), prevDist);
            prevDist = q.distance();
            ++q;
            n++;
        }
        ASSERT_EQ(Nq, n);
    }
}

}  // namespace phtree_test_const_values
