/*
 * Copyright 2022 Tilmann Zäschke
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

#include "phtree/common/bpt_fixed_vector.h"
#include <include/gtest/gtest.h>
#include <map>
#include <random>

using namespace phtree::bptree::detail;

static int default_construct_count_ = 0;
static int construct_count_ = 0;
static int copy_construct_count_ = 0;
static int move_construct_count_ = 0;
static int copy_assign_count_ = 0;
static int move_assign_count_ = 0;
static int destruct_count_ = 0;

[[maybe_unused]] static void reset_id_counters() {
    default_construct_count_ = 0;
    construct_count_ = 0;
    copy_construct_count_ = 0;
    move_construct_count_ = 0;
    copy_assign_count_ = 0;
    move_assign_count_ = 0;
    destruct_count_ = 0;
}

[[maybe_unused]] static void print_id_counters() {
    std::cout << "dc=" << default_construct_count_ << " c=" << construct_count_
              << " cc=" << copy_construct_count_ << " mc=" << move_construct_count_
              << " ca=" << copy_assign_count_ << " ma=" << move_assign_count_
              << " d=" << destruct_count_ << std::endl;
}

struct Id {
    Id() : i_{0} {
        ++default_construct_count_;
    }

    explicit Id(const size_t i) : i_{static_cast<int>(i)} {
        ++construct_count_;
    }

    explicit Id(const int i) : i_{i} {
        ++construct_count_;
    }

    Id(const Id& other) {
        ++copy_construct_count_;
        i_ = other.i_;
    }

    Id(Id&& other) noexcept {
        ++move_construct_count_;
        i_ = other.i_;
    }

    Id& operator=(const Id& other) noexcept {
        ++copy_assign_count_;
        i_ = other.i_;
        return *this;
    }
    Id& operator=(Id&& other) noexcept {
        ++move_assign_count_;
        i_ = other.i_;
        return *this;
    }

    bool operator==(const Id& rhs) const {
        return i_ == rhs.i_;
    }

    ~Id() {
        ++destruct_count_;
    }

    int i_;
};

struct IdTriviallyCopyable {
    IdTriviallyCopyable() : i_{0} {}

    explicit IdTriviallyCopyable(const size_t i) : i_{static_cast<int>(i)} {}

    explicit IdTriviallyCopyable(const int i) : i_{i} {}

    bool operator==(const IdTriviallyCopyable& rhs) const {
        return i_ == rhs.i_;
    }

    int i_;
};

TEST(PhTreeBptFixedVectorTest, SmokeTest0) {
    const size_t N = 100;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N - 1);

    using ValueT = Id;
    for (int i = 0; i < 100; i++) {
        bpt_vector<ValueT, N> test_map{};

        // populate 50%
        for (size_t j = 0; j < N / 2; j++) {
            test_map.emplace_back(j);
            // test_map._check();
        }
        ASSERT_EQ(test_map.size(), N / 2);

        // populate 100%
        for (size_t j = N / 2; j < N; j++) {
            test_map.emplace(test_map.begin() + N / 4, j);
        }
        ASSERT_EQ(test_map.size(), N);

        for (size_t j = 0; j < N; j++) {
            ASSERT_LE(test_map[j].i_, (int)N);
        }

        // update
        for (size_t j = 0; j < N; j++) {
            int pos = cube_distribution(random_engine);
            test_map.erase(test_map.begin() + pos);

            // add
            int pos2 = cube_distribution(random_engine);
            test_map.emplace(test_map.begin() + pos2, j);
        }
        ASSERT_EQ(test_map.size(), N);

        // update ranges
        for (size_t j = 0; j < N; j++) {
            int R = 5;
            size_t pos = cube_distribution(random_engine) % (N - R);
            test_map.erase(test_map.begin() + pos, test_map.begin() + pos + R);

            // add
            int pos2 = std::max(0, (int)(cube_distribution(random_engine) % N - R));
            bpt_vector<Id> tm2{};
            // std::vector<Id> ref2{};
            for (int k = 0; k < R; ++k) {
                tm2.emplace_back(j + k);
            }
            test_map.insert(
                test_map.begin() + pos2,
                std::move_iterator(tm2.begin()),
                std::move_iterator(tm2.end()));
        }
        ASSERT_EQ(test_map.size(), N);

        size_t n = 0;
        for (auto it = test_map.begin(); it != test_map.end(); ++it) {
            ++n;
        }
        ASSERT_EQ(N, n);
        static_assert(std::is_same_v<decltype(test_map.begin()), decltype(test_map.end())>);

        // drain 50%
        while (test_map.size() > N / 2) {
            size_t pos = cube_distribution(random_engine) % (N / 4);
            test_map.erase(test_map.begin() + pos, test_map.begin() + pos + 3);
        }

        // drain 100%
        while (!test_map.empty()) {
            size_t pos = cube_distribution(random_engine) % test_map.size();
            test_map.erase(test_map.begin() + pos);
        }

        ASSERT_EQ(0u, test_map.size());
        ASSERT_TRUE(test_map.empty());
    }

    ASSERT_GE(construct_count_ + copy_construct_count_ + move_construct_count_, destruct_count_);
    ASSERT_LE(construct_count_ + copy_construct_count_, destruct_count_);
}

template <typename Id>
void SmokeTest() {
    const size_t N = 100;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N - 1);

    using ValueT = Id;
    for (int i = 0; i < 100; i++) {
        bpt_vector<ValueT, N> test_map{};
        std::vector<ValueT> reference_map{};

        // populate 50%
        for (size_t j = 0; j < N / 2; j++) {
            test_map.emplace_back(j);
            // test_map._check();
            reference_map.emplace_back(j);
        }
        ASSERT_EQ(test_map.size(), reference_map.size());

        // populate 100%
        for (size_t j = N / 2; j < N; j++) {
            test_map.emplace(test_map.begin() + N / 4, j);
            // test_map._check();
            reference_map.emplace(reference_map.begin() + N / 4, j);

            ASSERT_EQ(test_map.size(), reference_map.size());
        }
        ASSERT_EQ(test_map.size(), reference_map.size());

        for (size_t j = 0; j < N; j++) {
            ASSERT_EQ(test_map[j], reference_map[j]);
        }

        // update
        for (size_t j = 0; j < N; j++) {
            int pos = cube_distribution(random_engine);
            ASSERT_EQ(test_map[pos], reference_map[pos]);
            test_map.erase(test_map.begin() + pos);
            reference_map.erase(reference_map.begin() + pos);
            ASSERT_EQ(test_map.back(), reference_map.back());

            // add
            int pos2 = cube_distribution(random_engine);
            test_map.emplace(test_map.begin() + pos2, j);
            reference_map.emplace(reference_map.begin() + pos2, j);
            ASSERT_EQ(test_map[pos2], reference_map[pos2]);
            ASSERT_EQ(test_map.back(), reference_map.back());

            ASSERT_EQ(test_map.size(), reference_map.size());
            ASSERT_EQ(test_map.size(), N);
        }
        ASSERT_EQ(test_map.size(), reference_map.size());

        // update ranges
        for (size_t j = 0; j < N; j++) {
            int R = 5;
            size_t pos = cube_distribution(random_engine) % (N - R);
            ASSERT_EQ(test_map[pos], reference_map[pos]);
            test_map.erase(test_map.begin() + pos, test_map.begin() + pos + R);
            reference_map.erase(reference_map.begin() + pos, reference_map.begin() + pos + R);
            ASSERT_EQ(test_map.back(), reference_map.back());

            // add
            int pos2 = std::max(0, (int)(cube_distribution(random_engine) % N - R));
            bpt_vector<Id> tm2{};
            std::vector<Id> ref2{};
            for (int k = 0; k < R; ++k) {
                tm2.emplace_back(j + k);
                ref2.emplace_back(j + k);
            }
            test_map.insert(
                test_map.begin() + pos2,
                std::move_iterator(tm2.begin()),
                std::move_iterator(tm2.end()));
            reference_map.insert(reference_map.begin() + pos2, ref2.begin(), ref2.end());
            ASSERT_EQ(test_map[pos2], reference_map[pos2]);
            ASSERT_EQ(test_map.back(), reference_map.back());

            ASSERT_EQ(test_map.size(), reference_map.size());
            ASSERT_EQ(test_map.size(), N);
        }
        ASSERT_EQ(test_map.size(), reference_map.size());

        size_t n = 0;
        for (auto it = test_map.begin(); it != test_map.end(); ++it) {
            ++n;
        }
        ASSERT_EQ(N, n);
        static_assert(std::is_same_v<decltype(test_map.begin()), decltype(test_map.end())>);

        // drain 50%
        while (test_map.size() > N / 2) {
            size_t pos = cube_distribution(random_engine) % (N / 4);
            test_map.erase(test_map.begin() + pos, test_map.begin() + pos + 3);
            reference_map.erase(reference_map.begin() + pos, reference_map.begin() + pos + 3);
            ASSERT_EQ(test_map.back(), reference_map.back());
        }

        // drain 100%
        while (!test_map.empty()) {
            size_t pos = cube_distribution(random_engine) % test_map.size();
            ASSERT_EQ(test_map[pos], reference_map[pos]);
            test_map.erase(test_map.begin() + pos);
            reference_map.erase(reference_map.begin() + pos);
        }

        ASSERT_EQ(0u, test_map.size());
        ASSERT_TRUE(test_map.empty());
    }
}

TEST(PhTreeBptFixedVectorTest, SmokeTest) {
    static_assert(!std::is_trivially_copyable_v<Id>);
    reset_id_counters();
    SmokeTest<Id>();
    ASSERT_GE(construct_count_ + copy_construct_count_ + move_construct_count_, destruct_count_);
    ASSERT_LE(construct_count_ + copy_construct_count_, destruct_count_);
}

TEST(PhTreeBptFixedVectorTest, SmokeTest_TriviallyCopyable) {
    static_assert(std::is_trivially_copyable_v<IdTriviallyCopyable>);
    SmokeTest<IdTriviallyCopyable>();
}

template <typename TREE>
void test_tree(TREE& tree) {
    // test various operations
    tree.emplace_back(Id(2));
    Id id3{3};
    tree.emplace_back(id3);
    ASSERT_EQ(tree.size(), 3u);

    tree.erase(tree.begin());
    ASSERT_EQ(2u, tree.size());
    tree.erase(tree.begin());
    tree.erase(tree.begin());
    ASSERT_EQ(0u, tree.size());
    ASSERT_TRUE(tree.empty());
}

TEST(PhTreeBptFixedVectorTest, TestCopyConstruct) {
    using TestTree = bpt_vector<Id>;
    TestTree tree1;
    tree1.emplace_back(Id(1));

    TestTree tree{tree1};
    test_tree(tree);
    // The old tree should still work!
    test_tree(tree1);
}

TEST(PhTreeBptFixedVectorTest, TestCopyAssign) {
    using TestTree = bpt_vector<Id>;
    TestTree tree1;
    tree1.emplace_back(Id(1));

    TestTree tree{};
    tree = tree1;
    test_tree(tree);
    // The old tree should still work!
    test_tree(tree1);
}

TEST(PhTreeBptFixedVectorTest, TestMoveConstruct) {
    using TestTree = bpt_vector<Id>;
    TestTree tree1;
    tree1.emplace_back(Id(1));

    TestTree tree{std::move(tree1)};
    test_tree(tree);
}

TEST(PhTreeBptFixedVectorTest, TestMoveAssign) {
    using TestTree = bpt_vector<Id>;
    TestTree tree1;
    tree1.emplace_back(Id(1));

    TestTree tree{};
    tree = std::move(tree1);
    test_tree(tree);
}
