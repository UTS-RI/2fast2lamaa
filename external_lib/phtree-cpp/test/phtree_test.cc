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

namespace phtree_test {

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

static int default_construct_count_ = 0;
static int construct_count_ = 0;
static int copy_construct_count_ = 0;
static int move_construct_count_ = 0;
static int copy_assign_count_ = 0;
static int move_assign_count_ = 0;
static int destruct_count_ = 0;

static void reset_id_counters() {
    default_construct_count_ = 0;
    construct_count_ = 0;
    copy_construct_count_ = 0;
    move_construct_count_ = 0;
    copy_assign_count_ = 0;
    move_assign_count_ = 0;
    destruct_count_ = 0;
}

static void print_id_counters() {
    std::cout << "dc=" << default_construct_count_ << " c=" << construct_count_
              << " cc=" << copy_construct_count_ << " mc=" << move_construct_count_
              << " ca=" << copy_assign_count_ << " ma=" << move_assign_count_
              << " d=" << destruct_count_ << std::endl;
}

struct Id {
    Id() : _i{0} {
        ++default_construct_count_;
    }

    explicit Id(const size_t i) : _i{static_cast<int>(i)} {
        ++construct_count_;
    }

    Id(const Id& other) {
        ++copy_construct_count_;
        _i = other._i;
    }

    Id(Id&& other) noexcept {
        ++move_construct_count_;
        _i = other._i;
    }

    Id& operator=(const Id& other) noexcept {
        ++copy_assign_count_;
        _i = other._i;
        return *this;
    }
    Id& operator=(Id&& other) noexcept {
        ++move_assign_count_;
        _i = other._i;
        return *this;
    }

    bool operator==(const Id& rhs) const {
        return _i == rhs._i;
    }

    ~Id() {
        ++destruct_count_;
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
double distance_L1(const TestPoint<DIM>& p1, const TestPoint<DIM>& p2) {
    double sum = 0;
    for (dimension_t i = 0; i < DIM; i++) {
        sum += std::abs(p1[i] - p2[i]);
    }
    return sum;
}

template <dimension_t DIM>
double distance_chebyshev(const TestPoint<DIM>& p1, const TestPoint<DIM>& p2) {
    double sum = 0;
    for (dimension_t i = 0; i < DIM; i++) {
        sum = std::max(sum, (double)std::abs(p1[i] - p2[i]));
    }
    return sum;
}

template <dimension_t DIM>
void generateCube(std::vector<TestPoint<DIM>>& points, size_t N) {
    IntRng rng(-1000, 1000);
    auto refTree = std::map<TestPoint<DIM>, size_t>();

    points.reserve(N);
    for (size_t i = 0; i < N; i++) {
        auto point = TestPoint<DIM>();
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

template <dimension_t DIM>
void SmokeTestBasicOps(size_t N) {
    reset_id_counters();
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
        if (i % 4 == 0) {
            ASSERT_TRUE(tree.emplace(p, i).second);
        } else if (i % 4 == 1) {
            ASSERT_TRUE(tree.insert(p, id).second);
        } else {
            ASSERT_TRUE(tree.try_emplace(p, i).second);
        }
        ASSERT_EQ(tree.count(p), 1);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(id._i, tree.find(p)->_i);
        ASSERT_EQ(i + 1, tree.size());

        // try insert/emplace again
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

    // Normal construction and destruction should be symmetric. Move-construction is ignored.
    ASSERT_GE(construct_count_ + copy_construct_count_ + move_construct_count_, destruct_count_);
    ASSERT_LE(construct_count_ + copy_construct_count_, destruct_count_);
    // The following assertions exist only as sanity checks and may need adjusting.
    // There is nothing fundamentally wrong if a change in the implementation violates
    // any of the following assertions, as long as performance/memory impact is observed.
    ASSERT_EQ(0, default_construct_count_);
    ASSERT_EQ(0, copy_assign_count_);
    ASSERT_EQ(0, move_assign_count_);
    // Only insert() should cause a copy, emplace() should not.
    ASSERT_GE(construct_count_ / 2., copy_construct_count_);
    if (DIM > 3 && DIM < 8) {
        // as expected the sparse array map does a _lot_ of copying (shifting entries around)
        ASSERT_GE(construct_count_ * 7, move_construct_count_);
    } else if (DIM == 1) {
        // small node require a lot of copying/moving
        ASSERT_GE(construct_count_ * 3, move_construct_count_);
    } else {
        if (construct_count_ * 15 < move_construct_count_) {
            print_id_counters();
        }
        ASSERT_GE(construct_count_ * 15, move_construct_count_);
    }
}

TEST(PhTreeTest, SmokeTestBasicOps) {
    SmokeTestBasicOps<1>(100);
    SmokeTestBasicOps<3>(10000);
    SmokeTestBasicOps<6>(10000);
    SmokeTestBasicOps<10>(10000);
    SmokeTestBasicOps<20>(10000);
    SmokeTestBasicOps<32>(1000);
    SmokeTestBasicOps<63>(100);
}

TEST(PhTreeTest, TestDebug) {
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

TEST(PhTreeTest, TestInsert) {
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

TEST(PhTreeTest, TestEmplace) {
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

TEST(PhTreeTest, TestSquareBrackets) {
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
            tree[p]._i = (int)i;
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
        ASSERT_EQ(i, tree[p]._i);
    }
}

template <dimension_t DIM>
void populate(TestTree<DIM, size_t>& tree, std::vector<TestPoint<DIM>>& points, size_t N) {
    generateCube(points, N);
    for (size_t i = 0; i < N; i++) {
        ASSERT_TRUE(tree.insert(points[i], i).second);
    }
    ASSERT_EQ(N, tree.size());
}

template <dimension_t DIM>
void populate(TestTree<DIM, Id>& tree, std::vector<TestPoint<DIM>>& points, size_t N) {
    generateCube(points, N);
    for (size_t i = 0; i < N; i++) {
        ASSERT_TRUE(tree.emplace(points[i], i).second);
    }
    ASSERT_EQ(N, tree.size());
}

TEST(PhTreeTest, TestClear) {
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

TEST(PhTreeTest, TestFind) {
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

TEST(PhTreeTest, TestLowerBound) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 200;
    std::vector<TestPoint<dim>> points;

    // find non-existing "lowest" point
    TestPoint<dim> pMin{0, 0, 0};
    // find non-existing somewhere in the middle
    TestPoint<dim> pMid{1000, 1000, 1000};
    // find non-existing "highest" point
    TestPoint<dim> pMax{-1, -1, -1};

    // empty tree
    ASSERT_EQ(tree.lower_bound(pMin), tree.end());
    ASSERT_EQ(tree.lower_bound(pMid), tree.end());
    ASSERT_EQ(tree.lower_bound(pMax), tree.end());

    populate(tree, points, N);

    size_t i = 0;
    size_t n_begin = 0;
    size_t n_end = 0;
    size_t n_not_end = 0;
    size_t n2 = 0;
    for (auto& p : points) {
        // test commutativity
        ASSERT_NE(tree.lower_bound(p), tree.end());
        ASSERT_NE(tree.end(), tree.lower_bound(p));
        ASSERT_EQ(tree.lower_bound(p)->_i, i);
        ASSERT_EQ(tree.lower_bound(p), tree.find(p));

        // test entry
        auto it = tree.lower_bound(p);
        n_begin += it == tree.begin();
        ++n2;
        ++it;
        n_end += it == tree.end();
        n_not_end += it != tree.end();
        while (it != tree.end()) {
            ++n2;
            ++it;
        }

        // min/max
        ASSERT_EQ(tree.lower_bound(pMin), tree.begin());
        ASSERT_EQ(tree.lower_bound(pMax), tree.end());

        ++i;
    }
    ASSERT_EQ(1, n_begin);
    ASSERT_EQ(1, n_end);
    ASSERT_EQ(N - 1, n_not_end);
    ASSERT_EQ(N * N / 2 + N / 2, n2);

    // min / mid / max
    ASSERT_EQ(tree.lower_bound(pMin), tree.begin());
    ASSERT_NE(tree.lower_bound(pMid), tree.begin());
    ASSERT_NE(tree.lower_bound(pMid), tree.end());
    ASSERT_EQ(tree.lower_bound(pMax), tree.end());

    auto iter1 = tree.lower_bound(points[0]);
    auto iter2 = tree.lower_bound(points[0]);
    ASSERT_EQ(iter1, iter2);
    ASSERT_NE(tree.end(), iter1);
}

TEST(PhTreeTest, TestLowerBoundErase) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 100;
    std::vector<TestPoint<dim>> points;

    populate(tree, points, N);

    for (size_t i = 0; i < N; ++i) {
        auto& p = points[i];
        // test entry
        auto it = tree.lower_bound(p);
        ASSERT_NE(it, tree.end());
        tree.erase(it);
        ASSERT_EQ(tree.find(p), tree.end());
    }
    ASSERT_EQ(tree.size(), 0);
}

TEST(PhTreeTest, TestLowerBoundEmplace) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 100;
    std::vector<TestPoint<dim>> points;
    generateCube(points, N);

    for (size_t i = 0; i < N; ++i) {
        auto& p = points[i];
        // test entry
        auto it = tree.lower_bound(p);
        tree.emplace_hint(it, p, i);
        ASSERT_NE(tree.find(p), tree.end());
    }
    ASSERT_EQ(tree.size(), N);

    // Verify
    for (size_t i = 0; i < N; ++i) {
        auto& p = points[i];
        ASSERT_NE(tree.find(p), tree.end());
        ASSERT_EQ(tree.find(p)->_i, i);
    }
}

TEST(PhTreeTest, TestUpdateWithEmplace) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
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

TEST(PhTreeTest, TestUpdateWithEmplaceHint) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::array<int, 4> deltas{0, 1, 10, 100};
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t d_n = 0;
    for (auto& p : points) {
        auto pOld = p;
        d_n = (d_n + 1) % deltas.size();
        int delta = deltas[d_n];
        TestPoint<dim> pNew{pOld[0] + delta, pOld[1] + delta, pOld[2] + delta};
        auto iter = tree.find(pOld);
        size_t n = tree.erase(iter);
        ASSERT_EQ(1u, n);
        tree.emplace_hint(iter, pNew, 42);
        ASSERT_EQ(1, tree.count(pNew));
        if (delta != 0.0) {
            ASSERT_EQ(0, tree.count(pOld));
        }
        p = pNew;
    }

    ASSERT_EQ(N, tree.size());
    tree.clear();

    tree.emplace_hint(tree.end(), {11, 21, 31}, 421);
    tree.emplace_hint(tree.begin(), {1, 2, 3}, 42);
    ASSERT_EQ(2, tree.size());
}

TEST(PhTreeTest, TestUpdateWithTryEmplaceHint) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::array<int, 4> deltas{0, 1, 10, 100};
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t d_n = 0;
    for (auto& p : points) {
        auto pOld = p;
        d_n = (d_n + 1) % deltas.size();
        int delta = deltas[d_n];
        TestPoint<dim> pNew{pOld[0] + delta, pOld[1] + delta, pOld[2] + delta};
        auto iter = tree.find(pOld);
        size_t n = tree.erase(iter);
        ASSERT_EQ(1u, n);
        tree.try_emplace(iter, pNew, 42);
        ASSERT_EQ(1, tree.count(pNew));
        if (delta != 0.0) {
            ASSERT_EQ(0, tree.count(pOld));
        }
        p = pNew;
    }

    ASSERT_EQ(N, tree.size());
    tree.clear();

    tree.try_emplace(tree.end(), {11, 21, 31}, 421);
    tree.try_emplace(tree.begin(), {1, 2, 3}, 42);
    ASSERT_EQ(2, tree.size());
}

TEST(PhTreeTest, TestUpdateWithRelocate) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::array<scalar_64_t, 4> deltas{0, 1, 10, 100};
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t d_n = 0;
    for (int x = 0; x < 10; ++x) {
        size_t i = 0;
        for (auto& p : points) {
            auto pOld = p;
            d_n = (d_n + 1) % deltas.size();
            scalar_64_t delta = deltas[d_n];
            TestPoint<dim> pNew{pOld[0] + delta, pOld[1] + delta, pOld[2] + delta};
            if (delta > 0.0 && tree.find(pNew) != tree.end()) {
                // Skip this, there is already another entry
                ASSERT_EQ(0, tree.relocate(pOld, pNew));
            } else {
                ASSERT_EQ(1, tree.relocate(pOld, pNew));
                if (delta > 0.0) {
                    // second time fails because value has already been moved
                    ASSERT_EQ(0, tree.relocate(pOld, pNew));
                }
                ASSERT_EQ(Id(i), *tree.find(pNew));
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
    tree.emplace(points[0], 1);
    ASSERT_EQ(1, tree.relocate(points[0], points[1]));
    ASSERT_EQ(tree.end(), tree.find(points[0]));
    ASSERT_EQ(Id(1), *tree.find(points[1]));
    ASSERT_EQ(1, tree.size());
    tree.clear();

    // check that existing destination fails
    tree.emplace(points[0], 1);
    tree.emplace(points[1], 2);
    ASSERT_EQ(0, tree.relocate(points[0], points[1]));
}

TEST(PhTreeTest, TestUpdateWithRelocateIf) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    size_t N = 10000;
    std::array<scalar_64_t, 4> deltas{0, 1, 10, 100};
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t d_n = 0;
    for (int x = 0; x < 10; ++x) {
        size_t i = 0;
        size_t done = 0;
        auto pred = [](const Id& id) { return id._i % 2 == 0; };
        for (auto& p : points) {
            auto pOld = p;
            d_n = (d_n + 1) % deltas.size();
            scalar_64_t delta = deltas[d_n];
            TestPoint<dim> pNew{pOld[0] + delta, pOld[1] + delta, pOld[2] + delta};
            if ((delta > 0.0 && tree.find(pNew) != tree.end()) || (i % 2 != 0)) {
                // Skip this, there is already another entry
                ASSERT_EQ(0, tree.relocate_if(pOld, pNew, pred));
            } else {
                ASSERT_EQ(1, tree.relocate_if(pOld, pNew, pred));
                if (delta > 0.0) {
                    // second time fails because value has already been moved
                    ASSERT_EQ(0, tree.relocate_if(pOld, pNew, pred));
                }
                ASSERT_EQ(Id(i), *tree.find(pNew));
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
    tree.emplace(points[0], 1);
    ASSERT_EQ(1, tree.relocate_if(points[0], points[1], pred));
    ASSERT_EQ(tree.end(), tree.find(points[0]));
    ASSERT_EQ(Id(1), *tree.find(points[1]));
    ASSERT_EQ(1, tree.size());
}

TEST(PhTreeTest, TestEraseByIterator) {
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
        if (i % 100 == 0 || tree.size() < 10) {
            PhTreeDebugHelper::CheckConsistency(tree);
        }
    }

    ASSERT_EQ(0, tree.erase(tree.end()));
}

TEST(PhTreeTest, TestEraseByIteratorQuery) {
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
        if (i % 100 == 0 || tree.size() < 10) {
            PhTreeDebugHelper::CheckConsistency(tree);
        }
    }

    ASSERT_EQ(0, tree.erase(tree.end()));
}

TEST(PhTreeTest, TestExtent) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
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

TEST(PhTreeTest, TestExtentFilter) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
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

TEST(PhTreeTest, TestRangeBasedForLoop) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
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

TEST(PhTreeTest, TestWindowQuery0) {
    const dimension_t dim = 3;
    TestPoint<dim> p{-10000, -10000, -10000};
    int n = 0;
    testQuery<dim>(p, p, 10000, n);
    ASSERT_EQ(0, n);
}

TEST(PhTreeTest, TestWindowQuery1) {
    size_t N = 1000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    int n = 0;
    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
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

TEST(PhTreeTest, TestWindowQuery1_WithFilter) {
    size_t N = 1000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    struct Counter {
        void operator()(TestPoint<dim>, Id& t) {
            ++n_;
            id_ = t;
        }
        Id id_{};
        size_t n_ = 0;
    };

    for (size_t i = 0; i < N; i++) {
        TestPoint<dim>& p = points.at(i);
        Counter callback{};
        FilterAABB filter(p, p, tree.converter());
        tree.for_each(callback, filter);
        ASSERT_EQ(i, callback.id_._i);
        ASSERT_EQ(1, callback.n_);
    }
}

TEST(PhTreeTest, TestWindowQueryMany) {
    const dimension_t dim = 3;
    TestPoint<dim> min{-100, -100, -100};
    TestPoint<dim> max{100, 100, 100};
    int n = 0;
    testQuery<dim>(min, max, 10000, n);
    ASSERT_LE(3, n);
    ASSERT_GE(100, n);
}

TEST(PhTreeTest, TestWindowQueryAll) {
    const dimension_t dim = 3;
    const size_t N = 10000;
    TestPoint<dim> min{-10000, -10000, -10000};
    TestPoint<dim> max{10000, 10000, 10000};
    int n = 0;
    testQuery<dim>(min, max, N, n);
    ASSERT_EQ(N, n);
}

TEST(PhTreeTest, TestWindowQueryManyMoving) {
    size_t N = 10000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
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

TEST(PhTreeTest, TestWindowForEachManyMoving) {
    size_t N = 10000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    int query_length = 200;
    size_t nn = 0;
    for (std::int64_t i = -120; i < 120; i++) {
        TestPoint<dim> min{i * 10, i * 9, i * 11};
        TestPoint<dim> max{i * 10 + query_length, i * 9 + query_length, i * 11 + query_length};
        std::set<size_t> referenceResult;
        referenceQuery(points, min, max, referenceResult);

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
        tree.for_each({min, max}, callback);
        n += callback.n_;
        nn += callback.n_;
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

TEST(PhTreeTest, TestWindowForEachExact) {
    size_t N = 1000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    size_t nn = 0;
    for (size_t i = 0; i < N; i++) {
        size_t n = 0;
        tree.for_each({points[i], points[i]}, [&](TestPoint<dim>, Id&) {
            ++n;
            ++nn;
        });
        ASSERT_EQ(1, n);
    }
    ASSERT_EQ(N, nn);
}

TEST(PhTreeTest, TestWindowQueryIterators) {
    size_t N = 1000;
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
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

TEST(PhTreeTest, TestWindowQueryFilter) {
    const dimension_t dim = 3;
    TestTree<dim, Id> tree;
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

template <typename DIST_TEST, typename DIST_REF>
void test_knn_query(DIST_TEST dist_fn, DIST_REF dist_fn_reference, bool check_id = false) {
    // deliberately allowing outside of main points range
    IntRng rng(-1500, 1500);
    const dimension_t dim = 3;
    const size_t N = 1000;
    const size_t Nq = 10;

    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    for (size_t round = 0; round < 100; round++) {
        TestPoint<dim> center{rng.next(), rng.next(), rng.next()};

        // sort points manually
        std::vector<PointDistance> sorted_data;
        for (size_t i = 0; i < points.size(); i++) {
            double dist = dist_fn_reference(center, points[i]);
            sorted_data.emplace_back(dist, i);
        }
        std::sort(sorted_data.begin(), sorted_data.end(), comparePointDistance);

        size_t n = 0;
        double prevDist = -1;
        auto q = tree.begin_knn_query(Nq, center, dist_fn);
        while (q != tree.end()) {
            ASSERT_EQ(sorted_data[n]._distance, q.distance());
            if (check_id) {
                // With L1 and Chebyshev distances are not unique and IDs may be reordered
                auto& e = *q;
                ASSERT_EQ(sorted_data[n]._id, e._i);
                ASSERT_EQ(points[sorted_data[n]._id], q.first());
                ASSERT_EQ(sorted_data[n]._id, q.second()._i);
            }
            ASSERT_GE(q.distance(), prevDist);
            prevDist = q.distance();
            ++q;
            n++;
        }
        ASSERT_EQ(Nq, n);
    }
}

TEST(PhTreeTest, TestKnnQuery_Euclidean) {
    const dimension_t DIM = 3;
    test_knn_query(
        DistanceEuclidean<3>(),
        [](const TestPoint<DIM>& v1, const TestPoint<DIM>& v2) { return distance(v1, v2); },
        true);
}

TEST(PhTreeTest, TestKnnQuery_L1) {
    const dimension_t DIM = 3;
    test_knn_query(DistanceL1<3>(), [](const TestPoint<DIM>& v1, const TestPoint<DIM>& v2) {
        return distance_L1(v1, v2);
    });
}

TEST(PhTreeTest, TestKnnQuery_Chebyshev) {
    const dimension_t DIM = 3;
    test_knn_query(DistanceChebyshev<3>(), [](const TestPoint<DIM>& v1, const TestPoint<DIM>& v2) {
        return distance_chebyshev(v1, v2);
    });
}

template <dimension_t DIM>
struct MyDistance {
    double operator()(const TestPoint<DIM>& v1, const TestPoint<DIM>& v2) const {
        return distance_L1(v1, v2);
    };
};

TEST(PhTreeTest, TestKnnQueryFilterAndCustomDistance) {
    // deliberately allowing outside of main points range
    IntRng rng(-1500, 1500);
    const dimension_t dim = 3;
    const size_t N = 100;
    const size_t Nq = 10;

    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    for (size_t round = 0; round < 100; round++) {
        TestPoint<dim> center{rng.next(), rng.next(), rng.next()};

        // sort points manually by L1; skip every 2nd point
        std::vector<PointDistance> sorted_data;
        for (size_t i = 0; i < points.size(); i += 2) {
            double dist = MyDistance<dim>{}(center, points[i]);
            sorted_data.emplace_back(dist, i);
        }
        std::sort(sorted_data.begin(), sorted_data.end(), comparePointDistance);

        size_t n = 0;
        double prevDist = -1;
        auto q = tree.begin_knn_query(Nq, center, MyDistance<dim>(), FilterEvenId<dim, Id>());
        while (q != tree.end()) {
            // just read the entry
            ASSERT_EQ(sorted_data[n]._distance, q.distance());
            // We don't check anything else because with L1 there will often be several different
            // entries with the same distance but with different ordering than sorted_data.
            ASSERT_GE(q.distance(), prevDist);
            prevDist = q.distance();
            ++q;
            n++;
        }
        ASSERT_EQ(Nq, n);
    }
}

TEST(PhTreeTest, TestKnnQueryIterator) {
    // deliberately allowing outside of main points range
    IntRng rng(-1500, 1500);
    const dimension_t dim = 3;
    const size_t N = 1000;
    const size_t Nq = 10;

    TestTree<dim, Id> tree;
    std::vector<TestPoint<dim>> points;
    populate(tree, points, N);

    TestPoint<dim> center{rng.next(), rng.next(), rng.next()};
    size_t n = 0;
    auto q1 = tree.begin_knn_query(Nq, center, DistanceEuclidean<3>());
    auto q2 = tree.begin_knn_query(Nq, center, DistanceEuclidean<3>());
    while (q1 != tree.end()) {
        ASSERT_NE(q1, tree.end());
        ASSERT_NE(q2, tree.end());
        ASSERT_EQ(q1, q2);
        ++q1;
        ASSERT_NE(q1, q2);
        ++q2;
        n++;
    }
    ASSERT_EQ(Nq, n);
}

TEST(PhTreeTest, SmokeTestPoint0) {
    // Test edge case: empty tree
    TestPoint<3> p{1, 2, 3};
    TestTree<3, Id> tree;
    ASSERT_EQ(tree.size(), 0);
    ASSERT_EQ(tree.find(p), tree.end());

    auto q_window = tree.begin_query({p, p});
    ASSERT_EQ(q_window, tree.end());

    auto q_extent = tree.begin();
    ASSERT_EQ(q_extent, tree.end());

    auto q_knn = tree.begin_knn_query(10, p, DistanceEuclidean<3>());
    ASSERT_EQ(q_knn, tree.end());

    ASSERT_EQ(0, tree.erase(p));
    ASSERT_EQ(0, tree.size());
    ASSERT_TRUE(tree.empty());
}

TEST(PhTreeTest, SmokeTestPoint1) {
    // Test edge case: only one entry in tree
    PhPoint<3> p{1, 2, 3};
    TestTree<3, Id> tree;
    tree.emplace(p, Id{1});
    tree.emplace(p, Id{2});
    Id id3{3};
    tree.insert(p, id3);
    Id id4{4};
    tree.insert(p, id4);
    ASSERT_EQ(tree.size(), 1);
    ASSERT_EQ(tree.find(p).second()._i, 1);
    ASSERT_EQ(tree[p]._i, 1);

    auto q_window = tree.begin_query({p, p});
    ASSERT_EQ(1, q_window->_i);
    ++q_window;
    ASSERT_EQ(q_window, tree.end());

    auto q_extent = tree.begin();
    ASSERT_EQ(1, q_extent->_i);
    ++q_extent;
    ASSERT_EQ(q_extent, tree.end());

    auto q_knn = tree.begin_knn_query(10, p, DistanceEuclidean<3>());
    ASSERT_EQ(1, q_knn->_i);
    ++q_knn;
    ASSERT_EQ(q_knn, tree.end());

    ASSERT_EQ(1, tree.erase(p));
    ASSERT_EQ(0, tree.size());
    ASSERT_EQ(0, tree.erase(p));
    ASSERT_EQ(0, tree.size());
    ASSERT_TRUE(tree.empty());
}

template <typename TREE>
void test_tree(TREE& tree) {
    PhPoint<3> p{1, 2, 3};

    // test various operations
    tree.emplace(p, Id{2});  // already exists
    Id id3{3};
    tree.insert(p, id3);  // already exists
    ASSERT_EQ(tree.size(), 1);
    ASSERT_EQ(tree.find(p).second()._i, 1);
    ASSERT_EQ(tree[p]._i, 1);

    auto q_window = tree.begin_query({p, p});
    ASSERT_EQ(1, q_window->_i);
    ++q_window;
    ASSERT_EQ(q_window, tree.end());

    auto q_extent = tree.begin();
    ASSERT_EQ(1, q_extent->_i);
    ++q_extent;
    ASSERT_EQ(q_extent, tree.end());

    auto q_knn = tree.begin_knn_query(10, p, DistanceEuclidean<3>());
    ASSERT_EQ(1, q_knn->_i);
    ++q_knn;
    ASSERT_EQ(q_knn, tree.end());

    ASSERT_EQ(1, tree.erase(p));
    ASSERT_EQ(0, tree.size());
    ASSERT_EQ(0, tree.erase(p));
    ASSERT_EQ(0, tree.size());
    ASSERT_TRUE(tree.empty());
}

TEST(PhTreeTest, TestMoveConstruct) {
    // Test edge case: only one entry in tree
    PhPoint<3> p{1, 2, 3};
    TestTree<3, Id> tree1;
    tree1.emplace(p, Id{1});

    TestTree<3, Id> tree{std::move(tree1)};
    test_tree(tree);
}

TEST(PhTreeTest, TestMoveAssign) {
    // Test edge case: only one entry in tree
    PhPoint<3> p{1, 2, 3};
    TestTree<3, Id> tree1;
    tree1.emplace(p, Id{1});

    TestTree<3, Id> tree{};
    tree = std::move(tree1);
    test_tree(tree);
}

size_t count_pre{0};
size_t count_post{0};
size_t count_query{0};

template <dimension_t DIM, typename SCALAR = scalar_64_t>
struct DebugConverterNoOp : public ConverterPointBase<DIM, SCALAR, SCALAR> {
    using BASE = ConverterPointBase<DIM, SCALAR, SCALAR>;
    using Point = typename BASE::KeyExternal;
    using PointInternal = typename BASE::KeyInternal;

    constexpr const PointInternal& pre(const Point& point) const {
        ++count_pre;
        ++const_cast<size_t&>(count_pre_local);
        return point;
    }

    constexpr const Point& post(const PointInternal& point) const {
        ++count_post;
        ++const_cast<size_t&>(count_post_local);
        return point;
    }

    constexpr const PhBox<DIM, SCALAR>& pre_query(const PhBox<DIM, SCALAR>& box) const {
        ++count_query;
        ++const_cast<size_t&>(count_query_local);
        return box;
    }

    size_t count_pre_local{0};
    size_t count_post_local{0};
    size_t count_query_local{0};
};

TEST(PhTreeTest, TestMoveAssignCustomConverter) {
    // Test edge case: only one entry in tree
    PhPoint<3> p{1, 2, 3};
    auto converter = DebugConverterNoOp<3>();
    auto tree1 = PhTree<3, Id, DebugConverterNoOp<3>>(converter);
    tree1.emplace(p, Id{1});
    ASSERT_GE(tree1.converter().count_pre_local, 1);
    ASSERT_EQ(tree1.converter().count_pre_local, count_pre);

    PhTree<3, Id, DebugConverterNoOp<3>> tree{};
    tree = std::move(tree1);
    // Assert that converter got moved (or copied?):
    ASSERT_GE(tree.converter().count_pre_local, 1);
    ASSERT_EQ(tree.converter().count_pre_local, count_pre);

    test_tree(tree);
    ASSERT_GE(tree.converter().count_pre_local, 2);
    ASSERT_EQ(tree.converter().count_pre_local, count_pre);
}

TEST(PhTreeTest, TestMovableIterators) {
    // Test edge case: only one entry in tree
    PhPoint<3> p{1, 2, 3};
    auto tree = TestTree<3, Id>();
    tree.emplace(p, Id{1});

    ASSERT_TRUE(std::is_move_constructible_v<decltype(tree.begin())>);
    ASSERT_TRUE(std::is_move_assignable_v<decltype(tree.begin())>);
    ASSERT_NE(tree.begin(), tree.end());

    ASSERT_TRUE(std::is_move_constructible_v<decltype(tree.end())>);
    ASSERT_TRUE(std::is_move_assignable_v<decltype(tree.end())>);

    ASSERT_TRUE(std::is_move_constructible_v<decltype(tree.find(p))>);
    ASSERT_TRUE(std::is_move_assignable_v<decltype(tree.find(p))>);
    ASSERT_NE(tree.find(p), tree.end());

    TestTree<3, Id>::QueryBox qb{{1, 2, 3}, {4, 5, 6}};
    FilterEvenId<3, Id> filter{};
    ASSERT_TRUE(std::is_move_constructible_v<decltype(tree.begin_query(qb, filter))>);
    // Not movable due to constant fields
    // ASSERT_TRUE(std::is_move_assignable_v<decltype(tree.begin_query(qb, filter))>);

    ASSERT_TRUE(std::is_move_constructible_v<decltype(tree.begin_knn_query(
                    3, {2, 3, 4}, DistanceEuclidean<3>()))>);
    // Not movable due to constant fields
    // ASSERT_TRUE(std::is_move_assignable_v<decltype(tree.begin_knn_query(
    //                3, {2, 3, 4}, DistanceEuclidean<3>()))>);
}

}  // namespace phtree_test
