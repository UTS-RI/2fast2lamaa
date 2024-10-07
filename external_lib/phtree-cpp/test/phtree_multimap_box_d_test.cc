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

namespace phtree_multimap_box_d_test {

// Number of entries that have the same coordinate
static const size_t NUM_DUPL = 4;
static const double WORLD_MIN = -1000;
static const double WORLD_MAX = 1000;
static const double BOX_LEN = 10;

template <dimension_t DIM>
using TestPoint = PhBoxD<DIM>;

template <dimension_t DIM, typename T>
using TestTree = PhTreeMultiMapBoxD<DIM, T>;

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

struct Id {
    Id() = default;

    explicit Id(const size_t i) : _i{static_cast<int>(i)}, data_{0} {}

    bool operator==(const Id& rhs) const {
        return _i == rhs._i;
    }

    int _i;
    int data_;
};
}  // namespace phtree_multimap_box_d_test

namespace std {
template <>
struct hash<phtree_multimap_box_d_test::Id> {
    size_t operator()(const phtree_multimap_box_d_test::Id& x) const {
        return std::hash<int>{}(x._i);
    }
};
};  // namespace std

namespace phtree_multimap_box_d_test {

struct PointDistance {
    PointDistance(double distance, size_t id) : _distance(distance), _id(static_cast<int>(id)) {}

    double _distance;
    int _id;
};

bool comparePointDistanceAndId(PointDistance& i1, PointDistance& i2) {
    return (i1._distance != i2._distance) ? (i1._distance < i2._distance) : (i1._id < i2._id);
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
void generateCube(std::vector<TestPoint<DIM>>& points, size_t N, double box_Len = BOX_LEN) {
    assert(N % NUM_DUPL == 0);
    DoubleRng rng(WORLD_MIN, WORLD_MAX);
    auto reference_set = std::unordered_map<TestPoint<DIM>, size_t>();

    points.reserve(N);
    for (size_t i = 0; i < N / NUM_DUPL; i++) {
        // create duplicates, i.e. entries with the same coordinates. However, avoid unintentional
        // duplicates.
        TestPoint<DIM> key{};
        for (dimension_t d = 0; d < DIM; ++d) {
            key.min()[d] = rng.next();
            key.max()[d] = key.min()[d] + box_Len;
        }
        if (reference_set.count(key) != 0) {
            i--;
            continue;
        }
        reference_set.emplace(key, i);
        for (size_t dupl = 0; dupl < NUM_DUPL; dupl++) {
            auto point = TestPoint<DIM>(key);
            points.push_back(point);
        }
    }
    ASSERT_EQ(reference_set.size(), N / NUM_DUPL);
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
        ASSERT_LE(tree.count(p), i % NUM_DUPL);
        if (i % NUM_DUPL == 0) {
            ASSERT_EQ(tree.end(), tree.find(p));
        }

        Id id(i);
        if (i % 4 == 0) {
            ASSERT_TRUE(tree.emplace(p, id).second);
        } else if (i % 4 == 1) {
            ASSERT_TRUE(tree.insert(p, id).second);
        } else {
            ASSERT_TRUE(tree.try_emplace(p, id).second);
        }
        ASSERT_EQ(tree.count(p), i % NUM_DUPL + 1);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(id._i, tree.find(p, id)->_i);
        ASSERT_EQ(i + 1, tree.size());

        // try adding it again
        ASSERT_FALSE(tree.insert(p, id).second);
        ASSERT_FALSE(tree.emplace(p, id).second);
        ASSERT_EQ(tree.count(p), i % NUM_DUPL + 1);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(id._i, tree.find(p, id)->_i);
        ASSERT_EQ(i + 1, tree.size());
        ASSERT_FALSE(tree.empty());
    }

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        // With intersection queries we may get multiple results.
        size_t found = 0;
        for (auto q = tree.begin_query(p); q != tree.end(); ++q) {
            found += (i / NUM_DUPL == (*q)._i / NUM_DUPL);
        }
        ASSERT_EQ(NUM_DUPL, found);
    }

    PhTreeDebugHelper::CheckConsistency(tree);

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        Id id(i);
        ASSERT_NE(tree.find(p), tree.end());
        size_t expected_remaining = (N - i - 1) % NUM_DUPL + 1;
        ASSERT_EQ(tree.count(p), expected_remaining);
        ASSERT_EQ(i, tree.find(p, id)->_i);
        if (i % 3 == 0) {
            ASSERT_EQ(1, tree.erase(p, id));
        } else {
            auto iter = tree.find(p, id);
            ASSERT_EQ(1, tree.erase(iter));
        }

        ASSERT_EQ(tree.count(p), expected_remaining - 1);
        if (expected_remaining - 1 == 0) {
            ASSERT_EQ(tree.end(), tree.find(p));
        }
        ASSERT_EQ(N - i - 1, tree.size());

        // try remove again
        ASSERT_EQ(0, tree.erase(p, id));
        ASSERT_EQ(tree.count(p), expected_remaining - 1);
        if (expected_remaining - 1 == 0) {
            ASSERT_EQ(tree.end(), tree.find(p));
        }
        ASSERT_EQ(N - i - 1, tree.size());
        if (i < N - 1) {
            ASSERT_FALSE(tree.empty());
        }
    }
    ASSERT_EQ(0, tree.size());
    ASSERT_TRUE(tree.empty());
    PhTreeDebugHelper::CheckConsistency(tree);
}

TEST(PhTreeMMBoxDTest, SmokeTestBasicOps) {
    SmokeTestBasicOps<1>(100);
    SmokeTestBasicOps<3>(10000);
    SmokeTestBasicOps<6>(10000);
    SmokeTestBasicOps<10>(1000);
    SmokeTestBasicOps<20>(1000);
    SmokeTestBasicOps<31>(100);
}

TEST(PhTreeMMBoxDTest, TestDebug) {
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
    ASSERT_EQ(N / NUM_DUPL, Debug::GetStats(tree).size_);
    Debug::CheckConsistency(tree);

    tree.clear();

    ASSERT_LE(10, Debug::ToString(tree, Debug::PrintDetail::name).length());
    ASSERT_GE(10, Debug::ToString(tree, Debug::PrintDetail::entries).length());
    ASSERT_GE(100, Debug::ToString(tree, Debug::PrintDetail::tree).length());
    ASSERT_EQ(0, Debug::GetStats(tree).size_);
    Debug::CheckConsistency(tree);
}

TEST(PhTreeMMBoxDTest, TestInsert) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 1000;

    std::vector<TestPoint<dim>> points;
    generateCube(points, N);

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        Id id(i);
        ASSERT_EQ(true, tree.insert(p, id).second);
        ASSERT_EQ(tree.count(p), i % NUM_DUPL + 1);
        ASSERT_EQ(id._i, tree.find(p, id)->_i);

        // try add again
        ASSERT_EQ(false, tree.insert(p, id).second);
        ASSERT_EQ(i, tree.insert(p, id).first._i);
        ASSERT_EQ(tree.count(p), i % NUM_DUPL + 1);
        ASSERT_EQ(id._i, tree.find(p, id)->_i);
    }
    ASSERT_EQ(N, tree.size());

    for (size_t i = 0; i < N; i++) {
        PhBoxD<dim>& p = points.at(i);
        // With intersection queries we may get more than NUM_DUPL results.
        size_t found = 0;
        for (auto q = tree.begin_query(p); q != tree.end(); ++q) {
            found += (i / NUM_DUPL == (*q)._i / NUM_DUPL);
        }
        ASSERT_EQ(NUM_DUPL, found);
    }

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        ASSERT_EQ(tree.count(p), NUM_DUPL);
        Id id(i);
        ASSERT_EQ(i, tree.find(p, id)->_i);
        ASSERT_EQ(i / NUM_DUPL, tree.find(p)->_i / NUM_DUPL);
    }
}

TEST(PhTreeMMBoxDTest, TestEmplace) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 1000;

    std::vector<TestPoint<dim>> points;
    generateCube(points, N);

    for (int i = 0; i < (int)N; i++) {
        TestPoint<dim>& p = points.at(i);
        Id id(i);
        ASSERT_EQ(true, tree.emplace(p, id).second);
        ASSERT_EQ(tree.count(p), i % NUM_DUPL + 1);
        ASSERT_EQ(id._i, tree.find(p, id)->_i);
        ASSERT_EQ(i + 1, tree.size());

        // try add again (same `identity`), this should NOT replace the existing value
        Id id2(i);
        id2.data_ = 42;
        ASSERT_EQ(false, tree.emplace(p, id2).second);
        ASSERT_EQ(i, tree.find(p, id2)->_i);
        ASSERT_EQ(0, tree.find(p, id2)->data_);
        ASSERT_EQ(i, tree.emplace(p, id).first._i);
        ASSERT_EQ(tree.count(p), i % NUM_DUPL + 1);

        // Check that the returned value is a reference
        tree.emplace(p, id2).first.data_++;
        ASSERT_EQ(1, tree.find(p, id)->data_);
        tree.emplace(p, id2).first.data_ = 0;
        ASSERT_EQ(0, tree.emplace(p, id).first.data_);
    }
    ASSERT_EQ(N, tree.size());

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        // With intersection queries we may get more than NUM_DUPL results.
        size_t found = 0;
        for (auto q = tree.begin_query(p); q != tree.end(); ++q) {
            found += (i / NUM_DUPL == (*q)._i / NUM_DUPL);
        }
        ASSERT_EQ(NUM_DUPL, found);
    }

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        Id id(i);
        ASSERT_EQ(tree.count(p), NUM_DUPL);
        ASSERT_EQ(i, tree.find(p, id)->_i);
    }
}

template <dimension_t DIM>
void populate(
    TestTree<DIM, size_t>& tree,
    std::vector<TestPoint<DIM>>& points,
    size_t N,
    double box_len = BOX_LEN) {
    generateCube(points, N, box_len);
    for (size_t i = 0; i < N; i++) {
        ASSERT_TRUE(tree.insert(points[i], i).second);
    }
    ASSERT_EQ(N, tree.size());
}

template <dimension_t DIM>
void populate(
    TestTree<DIM, Id>& tree,
    std::vector<TestPoint<DIM>>& points,
    size_t N,
    double box_len = BOX_LEN) {
    generateCube(points, N, box_len);
    for (size_t i = 0; i < N; i++) {
        ASSERT_TRUE(tree.emplace(points[i], (int)i).second);
    }
    ASSERT_EQ(N, tree.size());
}

TEST(PhTreeMMBoxDTest, TestClear) {
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

TEST(PhTreeMMBoxDTest, TestFind) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t i = 0;
    for (auto& p : points) {
        // test commutativity
        Id id(i);
        ASSERT_NE(tree.find(p), tree.end());
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_NE(tree.find(p, id), tree.end());
        ASSERT_NE(tree.end(), tree.find(p, id));
        ASSERT_EQ(tree.find(p, id)->_i, i);
        auto iterN = tree.find(points[0]);
        size_t n = 0;
        while (iterN != tree.end()) {
            ++iterN;
            ++n;
        }
        ASSERT_EQ(n, NUM_DUPL);
        i++;
    }

    TestPoint<dim> p({1, 1, 1}, {2, 2, 10000000});
    auto result = tree.find(p);
    ASSERT_EQ(result, tree.end());
    ASSERT_EQ(tree.end(), result);

    auto iter1 = tree.find(points[0]);
    auto iter2 = tree.find(points[0]);
    ASSERT_EQ(iter1, iter2);
    ASSERT_NE(tree.end(), iter1);
}

TEST(PhTreeMMBoxDTest, TestUpdateWithEmplace) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    double delta = 20;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t i = 0;
    for (auto& p : points) {
        auto pOld = p;
        TestPoint<dim> pNew(
            {pOld.min()[0] + delta, pOld.min()[1] + delta, pOld.min()[2] + delta},
            {pOld.max()[0] + delta, pOld.max()[1] + delta, pOld.max()[2] + delta});
        size_t count_new = tree.count(pNew);
        size_t count_old = tree.count(pOld);
        size_t n = tree.erase(pOld, Id(i));
        ASSERT_EQ(1U, n);
        tree.emplace(pNew, Id(i));
        ASSERT_EQ(count_new + 1, tree.count(pNew));
        ASSERT_EQ(count_old - 1, tree.count(pOld));
        p = pNew;
        ++i;
    }

    ASSERT_EQ(N, tree.size());
    tree.clear();
}

TEST(PhTreeMMBoxDTest, TestUpdateWithEmplaceHint) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::array<double, 4> deltas{0, 0.1, 1, 10};
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    int i = 0;
    size_t d_n = 0;
    for (auto& p : points) {
        auto pOld = p;
        d_n = (d_n + 1) % deltas.size();
        double delta = deltas[d_n];
        PhPointD<dim> min{pOld.min()[0] + delta, pOld.min()[1] + delta, pOld.min()[2] + delta};
        PhPointD<dim> max{pOld.max()[0] + delta, pOld.max()[1] + delta, pOld.max()[2] + delta};
        TestPoint<dim> pNew{min, max};
        auto iter = tree.find(pOld, Id(i));
        size_t n = tree.erase(iter);
        ASSERT_EQ(1U, n);
        ASSERT_TRUE(tree.emplace_hint(iter, pNew, Id(i)).second);
        ASSERT_EQ(Id(i), *tree.find(pNew, Id(i)));
        auto iterNew = tree.find(pNew, Id(i));
        ASSERT_FALSE(tree.emplace_hint(iterNew, pNew, Id(i)).second);
        p = pNew;
        ++i;
    }

    ASSERT_EQ(N, tree.size());
    tree.clear();
}

void TestUpdateWithRelocate(bool relocate_to_existing_coordinate) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::array<double, 4> deltas{0, 0.1, 1, 10};
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    for (auto delta : deltas) {
        size_t i = 0;
        for (auto& p : points) {
            auto pOld = p;
            TestPoint<dim> pNew;
            if (relocate_to_existing_coordinate) {
                pNew = delta > 0.0 ? points[(i + 17) % N] : pOld;
            } else {
                PhPointD<dim> min{
                    pOld.min()[0] + delta, pOld.min()[1] + delta, pOld.min()[2] + delta};
                PhPointD<dim> max{
                    pOld.max()[0] + delta, pOld.max()[1] + delta, pOld.max()[2] + delta};
                pNew = {min, max};
            }
            ASSERT_EQ(1u, tree.relocate(pOld, pNew, Id(i)));
            if (pOld != pNew) {
                // second time fails because value has already been moved
                ASSERT_EQ(0u, tree.relocate(pOld, pNew, Id(i)));
                ASSERT_EQ(tree.end(), tree.find(pOld, Id(i)));
            } else {
                ASSERT_EQ(1u, tree.relocate(pOld, pNew, Id(i)));
            }
            ASSERT_EQ(Id(i), *tree.find(pNew, Id(i)));
            p = pNew;
            ++i;
        }
        PhTreeDebugHelper::CheckConsistency(tree);
    }

    ASSERT_EQ(N, tree.size());
    tree.clear();
}

TEST(PhTreeMMBoxDTest, TestUpdateWithRelocateDelta) {
    TestUpdateWithRelocate(false);
}

TEST(PhTreeMMBoxDTest, TestUpdateWithRelocateToExisting) {
    TestUpdateWithRelocate(true);
}

TEST(PhTreeMMBoxDTest, TestUpdateWithRelocateCornerCases) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    TestPoint<dim> point0{{1, 2, 3}, {2, 3, 4}};
    TestPoint<dim> point1{{2, 3, 4}, {3, 4, 5}};

    // Check that empty tree works
    ASSERT_EQ(0u, tree.relocate(point0, point1, Id(42)));

    // Check that small tree works
    tree.emplace(point0, Id(1));
    ASSERT_EQ(1u, tree.relocate(point0, point1, Id(1)));
    ASSERT_EQ(tree.end(), tree.find(point0, Id(1)));
    ASSERT_EQ(1, tree.find(point1, Id(1))->_i);
    ASSERT_EQ(1u, tree.size());
    PhTreeDebugHelper::CheckConsistency(tree);
    tree.clear();

    // check that existing destination fails
    tree.emplace(point0, Id(1));
    tree.emplace(point1, Id(1));
    ASSERT_EQ(0u, tree.relocate(point0, point1, Id(1)));
    PhTreeDebugHelper::CheckConsistency(tree);
    tree.clear();

    // check that missing source bucket fails
    tree.emplace(point1, Id(1));
    ASSERT_EQ(0u, tree.relocate(point0, point1, Id(0)));
    PhTreeDebugHelper::CheckConsistency(tree);
    tree.clear();

    // check that missing source value fails (target bucket exists)
    tree.emplace(point0, Id(0));
    tree.emplace(point1, Id(1));
    ASSERT_EQ(0u, tree.relocate(point0, point1, Id(2)));
    PhTreeDebugHelper::CheckConsistency(tree);
    tree.clear();

    // check that missing source value fails (target bucket missing)
    tree.emplace(point0, Id(0));
    ASSERT_EQ(0u, tree.relocate(point0, point1, Id(2)));
    PhTreeDebugHelper::CheckConsistency(tree);
}

TEST(PhTreeMMBoxDTest, TestEraseByIterator) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t i = 0;
    for (auto& p : points) {
        ASSERT_NE(tree.end(), tree.find(p));
        auto iter = tree.find(p, Id(i));
        ASSERT_NE(tree.end(), iter);
        size_t count = tree.erase(iter);
        ASSERT_EQ(1U, count);
        ASSERT_EQ(tree.end(), tree.find(p, Id(i)));
        if (tree.size() % NUM_DUPL == 0) {
            ASSERT_EQ(tree.end(), tree.find(p));
        }
        i++;
    }

    ASSERT_EQ(0, tree.erase(tree.end()));
}

TEST(PhTreeMMBoxDTest, TestEraseByIteratorQuery) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    for (size_t i = 0; i < N; ++i) {
        auto iter = tree.begin();
        ASSERT_NE(tree.end(), iter);
        size_t count = tree.erase(iter);
        ASSERT_EQ(1U, count);
    }

    ASSERT_EQ(0, tree.erase(tree.end()));
}

TEST(PhTreeMMBoxDTest, TestExtent) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    int num_e = 0;
    auto qE = tree.begin();
    while (qE != tree.end()) {
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
    template <typename BucketT>
    [[nodiscard]] constexpr bool IsEntryValid(const PhPoint<2 * DIM>&, const BucketT&) const {
        return true;
    }
    [[nodiscard]] constexpr bool IsNodeValid(const PhPoint<2 * DIM>&, int) const {
        return true;
    }
    [[nodiscard]] constexpr bool IsBucketEntryValid(const PhPoint<2 * DIM>&, const T& value) const {
        return value._i % 2 == 0;
    }
};

TEST(PhTreeMMBoxDTest, TestExtentFilter) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    int num_e = 0;
    auto qE = tree.begin(FilterEvenId<dim, Id>());
    while (qE != tree.end()) {
        ASSERT_TRUE(qE->_i % 2 == 0);
        qE++;
        num_e++;
    }
    ASSERT_EQ(N, num_e * 2);
}

TEST(PhTreeMMBoxDTest, TestExtentForEachFilter) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    struct Counter {
        void operator()(const TestPoint<dim> key, const Id& t) {
            ++n_;
            ASSERT_EQ(points_[t._i], key);
            ASSERT_TRUE(t._i % 2 == 0);
        }
        std::vector<TestPoint<dim>>& points_;
        size_t n_ = 0;
    };
    Counter callback{points, 0};
    tree.for_each(callback, FilterEvenId<dim, Id>());
    ASSERT_EQ(N, callback.n_ * 2);
}

TEST(PhTreeMMBoxDTest, TestRangeBasedForLoop) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t num_e1 = 0;
    for (auto& x : tree) {
        x.data_ = 42;
        num_e1++;
    }
    ASSERT_EQ(N, num_e1);

    // Check that we really had references and that data_ was changed
    size_t num_e2 = 0;
    for (auto& x : tree) {
        ASSERT_EQ(42, x.data_);
        num_e2++;
    }
    ASSERT_EQ(N, num_e2);
}

TEST(PhTreeMMBoxDTest, TestEstimateCountIntersect) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 1000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    // Test small
    for (auto& p : points) {
        size_t n = tree.estimate_count(p);
        ASSERT_LE(NUM_DUPL, n);
        // arbitrary upper limit: 10*NUM_DUPL
        ASSERT_GE(10, NUM_DUPL);
    }

    // Test medium (1/8 of volume), allow variation of 20% 0.8 / 1.2
    double min_2 = WORLD_MIN / 2;
    double max_2 = WORLD_MAX / 2;
    size_t n_medium = tree.estimate_count({{min_2, min_2, min_2}, {max_2, max_2, max_2}});
    ASSERT_LE(N / 8. * 0.8, n_medium);
    ASSERT_GE(N / 8. * 1.2, n_medium);

    // Test all
    size_t n_all =
        tree.estimate_count({{WORLD_MIN, WORLD_MIN, WORLD_MIN}, {WORLD_MAX, WORLD_MAX, WORLD_MAX}});
    ASSERT_EQ(N, n_all);
}

TEST(PhTreeMMBoxDTest, TestEstimateCountInclude) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 1000;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    // Test small
    for (auto& p : points) {
        size_t n = tree.estimate_count(p, QueryInclude());
        ASSERT_LE(NUM_DUPL, n);
        // arbitrary upper limit: 10*NUM_DUPL
        ASSERT_GE(10, NUM_DUPL);
    }

    // Test medium (1/8 of volume), allow variation of 20% 0.8 / 1.2
    double min_2 = WORLD_MIN / 2;
    double max_2 = WORLD_MAX / 2;
    size_t n_medium =
        tree.estimate_count({{min_2, min_2, min_2}, {max_2, max_2, max_2}}, QueryInclude());
    ASSERT_LE(N / 8. * 0.8, n_medium);
    ASSERT_GE(N / 8. * 1.2, n_medium);

    // Test all
    double min_all = WORLD_MIN - BOX_LEN;
    double max_all = WORLD_MAX + BOX_LEN;
    size_t n_all = tree.estimate_count(
        {{min_all, min_all, min_all}, {max_all, max_all, max_all}}, QueryInclude());
    ASSERT_EQ(N, n_all);
}

template <dimension_t DIM>
void referenceQuery(
    std::vector<TestPoint<DIM>>& points,
    PhPointD<DIM>& min,
    PhPointD<DIM>& max,
    std::set<size_t>& result) {
    for (size_t i = 0; i < points.size(); i++) {
        auto& p = points[i];
        bool match = true;
        for (dimension_t d = 0; d < DIM; d++) {
            match &= p.max()[d] >= min[d] && p.min()[d] <= max[d];
        }
        if (match) {
            result.insert(i);
        }
    }
}

// We use 'int&' because gtest does not compile with assertions in non-void functions.
template <dimension_t DIM>
void testQuery(PhPointD<DIM>& min, PhPointD<DIM>& max, size_t N, int& result) {
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

TEST(PhTreeMMBoxDTest, TestWindowQuery0) {
    const dimension_t dim = 3;
    PhPointD<dim> p{-10000, -10000, -10000};
    int n = 0;
    testQuery<dim>(p, p, 10000, n);
    ASSERT_EQ(0, n);
}

TEST(PhTreeMMBoxDTest, TestWindowQuery1) {
    size_t N = 1000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t n = 0;
    for (size_t i = 0; i < N; i++) {
        PhBoxD<dim>& p = points.at(i);
        // With intersection queries we may get more than NUM_DUPL results.
        size_t found = 0;
        for (auto q = tree.begin_query(p); q != tree.end(); ++q) {
            found += (i / NUM_DUPL == (*q)._i / NUM_DUPL);
        }
        ASSERT_EQ(NUM_DUPL, found);
        n++;
    }
    ASSERT_EQ(N, n);
}

TEST(PhTreeMMBoxDTest, TestWindowQueryMany) {
    const dimension_t dim = 3;
    PhPointD<dim> min{-100, -100, -100};
    PhPointD<dim> max{100, 100, 100};
    int n = 0;
    testQuery<dim>(min, max, 10000, n);
    ASSERT_LE(3, n);
    ASSERT_GE(100, n);
}

TEST(PhTreeMMBoxDTest, TestWindowQueryAll) {
    const dimension_t dim = 3;
    const size_t N = 10000;
    PhPointD<dim> min{-10000, -10000, -10000};
    PhPointD<dim> max{10000, 10000, 10000};
    int n = 0;
    testQuery<dim>(min, max, N, n);
    ASSERT_EQ(N, n);
}

TEST(PhTreeMMBoxDTest, TestWindowQueryManyMoving) {
    size_t N = 10000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    double query_length = 200;
    size_t nn = 0;
    for (int i = -120; i < 120; i++) {
        PhPointD<dim> min{i * 10., i * 9., i * 11.};
        PhPointD<dim> max{i * 10. + query_length, i * 9. + query_length, i * 11. + query_length};
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
        ASSERT_GE(100, n);
    }
    ASSERT_LE(500, nn);
    ASSERT_GE(5000, nn);
}

TEST(PhTreeMMBoxDTest, TestWindowQueryManyMovingPoint) {
    size_t N = 10000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N, 100);

    size_t nTotal = 0;
    for (int i = -120; i < 120; i++) {
        PhPointD<dim> min_max{i * 10., i * 9., i * 11.};
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

TEST(PhTreeMMBoxDTest, TestWindowForEachManyMovingPoint) {
    size_t N = 10000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    std::vector<PhBoxD<dim>> points;
    populate(tree, points, N, 100);

    size_t nTotal = 0;
    for (int i = -120; i < 120; i++) {
        PhPointD<dim> min_max{i * 10., i * 9., i * 11.};
        std::set<size_t> referenceResult;
        referenceQuery(points, min_max, min_max, referenceResult);

        struct Counter {
            void operator()(const TestPoint<dim>&, const Id& t) {
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

TEST(PhTreeMMBoxDTest, SmokeTestPointAPI) {
    PhBoxD<3> p({1, 2, 3}, {4, 5, 6});
    (void)p;
}

TEST(PhTreeMMBoxDTest, SmokeTestTreeAPI) {
    std::map<int, Id*> mapPtr;
    PhTreeMultiMapBoxD<3, Id*> treePtr;
    Id* idPtr = new Id(1);
    treePtr.emplace(PhBoxD<3>({1, 2, 3}, {4, 5, 6}), idPtr);
    treePtr.clear();
    delete idPtr;
}

}  // namespace phtree_multimap_box_d_test
