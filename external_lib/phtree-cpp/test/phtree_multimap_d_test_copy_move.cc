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

namespace phtree_multimap_d_test_copy_move {

// Number of entries that have the same coordinate
static const size_t NUM_DUPL = 4;
static const double WORLD_MIN = -1000;
static const double WORLD_MAX = 1000;

template <dimension_t DIM>
using TestPoint = PhPointD<DIM>;

template <dimension_t DIM, typename T>
using TestTree = PhTreeMultiMap<DIM, T, ConverterIEEE<DIM>>;

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
    explicit IdCopyOnly(const size_t i) : _i{static_cast<int>(i)} {}

    IdCopyOnly() = default;
    IdCopyOnly(const IdCopyOnly& other) = default;
    IdCopyOnly(IdCopyOnly&& other) = delete;
    IdCopyOnly& operator=(const IdCopyOnly& other) = default;
    IdCopyOnly& operator=(IdCopyOnly&& other) = delete;
    ~IdCopyOnly() = default;

    bool operator==(const IdCopyOnly& rhs) const {
        return _i == rhs._i;
    }

    int _i{};
    int _data{};
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
    int _data{};
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
    int _data{};
};
}

namespace std {
template <>
struct hash<phtree_multimap_d_test_copy_move::IdCopyOnly> {
    size_t operator()(const phtree_multimap_d_test_copy_move::IdCopyOnly& x) const {
        return std::hash<int>{}(x._i);
    }
};
template <>
struct hash<phtree_multimap_d_test_copy_move::IdMoveOnly> {
    size_t operator()(const phtree_multimap_d_test_copy_move::IdMoveOnly& x) const {
        return std::hash<size_t>{}(x._i);
    }
};
template <>
struct hash<phtree_multimap_d_test_copy_move::IdCopyOrMove> {
    size_t operator()(const phtree_multimap_d_test_copy_move::IdCopyOrMove& x) const {
        return std::hash<size_t>{}(x._i);
    }
};
};  // namespace std

namespace phtree_multimap_d_test_copy_move {

struct IdHash {
    template <class T1, class T2>
    std::size_t operator()(std::pair<T1, T2> const& v) const {
        return std::hash<T1>()(v.size());
    }
};

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
            points.push_back(point);
        }
    }
    ASSERT_EQ(reference_set.size(), N / NUM_DUPL);
    ASSERT_EQ(points.size(), N);
}

template <dimension_t DIM, typename Id>
void SmokeTestBasicOps_QueryAndErase(TestTree<DIM, Id>& tree, std::vector<TestPoint<DIM>>& points) {
    size_t N = points.size();

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        auto q = tree.begin_query({p, p});
        ASSERT_NE(q, tree.end());
        for (size_t j = 0; j < NUM_DUPL; j++) {
            ASSERT_EQ(i / NUM_DUPL, (*q)._i / NUM_DUPL);
            ++q;
        }
        ASSERT_EQ(q, tree.end());
    }

    PhTreeDebugHelper::CheckConsistency(tree);

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        Id id(i);
        ASSERT_NE(tree.find(p), tree.end());
        size_t expected_remaining = (N - i - 1) % NUM_DUPL + 1;
        ASSERT_EQ(tree.count(p), expected_remaining);
        ASSERT_EQ(i, tree.find(p, id)->_i);
        if (i % 2 == 0) {
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

template <dimension_t DIM, typename Id>
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

    SmokeTestBasicOps_QueryAndErase(tree, points);
}

TEST(PhTreeMMDTestCopyMove, SmokeTestBasicOps) {
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

    ASSERT_EQ(0, tree.size());
    ASSERT_TRUE(tree.empty());
    PhTreeDebugHelper::CheckConsistency(tree);

    for (size_t i = 0; i < N; i++) {
        TestPoint<DIM>& p = points.at(i);
        ASSERT_LE(tree.count(p), i % NUM_DUPL);
        if (i % NUM_DUPL == 0) {
            ASSERT_EQ(tree.end(), tree.find(p));
        }

        if (i % 2 == 0) {
            ASSERT_TRUE(tree.emplace(p, Id(i)).second);
        } else {
            ASSERT_TRUE(tree.try_emplace(p, Id(i)).second);
        }
        ASSERT_EQ(tree.count(p), i % NUM_DUPL + 1);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(i, tree.find(p, Id(i))->_i);
        ASSERT_EQ(i + 1, tree.size());

        // try adding it again
        ASSERT_FALSE(tree.try_emplace(p, Id(i)).second);
        ASSERT_FALSE(tree.emplace(p, Id(i)).second);
        ASSERT_EQ(tree.count(p), i % NUM_DUPL + 1);
        ASSERT_NE(tree.end(), tree.find(p));
        ASSERT_EQ(i, tree.find(p, Id(i))->_i);
        ASSERT_EQ(i + 1, tree.size());
        ASSERT_FALSE(tree.empty());
    }

    SmokeTestBasicOps_QueryAndErase(tree, points);
}

TEST(PhTreeMMDTestCopyMove, SmokeTestBasicOpsMoveOnly) {
    SmokeTestBasicOpsMoveOnly<1, IdMoveOnly>(100);
    SmokeTestBasicOpsMoveOnly<3, IdMoveOnly>(100);
    SmokeTestBasicOpsMoveOnly<6, IdMoveOnly>(100);
    SmokeTestBasicOpsMoveOnly<10, IdMoveOnly>(100);
    SmokeTestBasicOpsMoveOnly<20, IdMoveOnly>(100);
    SmokeTestBasicOpsMoveOnly<63, IdMoveOnly>(100);
}

TEST(PhTreeMMDTestCopyMove, SmokeTestBasicOpsCopyFails) {
    SmokeTestBasicOpsMoveOnly<1, IdCopyOrMove>(100);
    SmokeTestBasicOpsMoveOnly<3, IdCopyOrMove>(100);
    SmokeTestBasicOpsMoveOnly<6, IdCopyOrMove>(100);
    SmokeTestBasicOpsMoveOnly<10, IdCopyOrMove>(100);
    SmokeTestBasicOpsMoveOnly<20, IdCopyOrMove>(100);
    SmokeTestBasicOpsMoveOnly<63, IdCopyOrMove>(100);
}

}  // namespace phtree_multimap_d_test_copy_move
