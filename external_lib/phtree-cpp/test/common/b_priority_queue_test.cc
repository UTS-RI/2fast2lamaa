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

#include "phtree/common/bpt_priority_queue.h"
#include <include/gtest/gtest.h>
#include <map>
#include <queue>
#include <random>

using namespace phtree::bptree;

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

template <typename Key, typename Value>
void populate(
    const size_t N,
    detail::priority_queue<Value>& test_map,
    std::multimap<Key, Value>& reference_map,
    std::vector<std::pair<Value, Key>>& reverse_map,
    std::default_random_engine& random_engine) {
    std::uniform_int_distribution<> cube_distribution(0, (int)N / 2);
    for (size_t j = 0; j < N; j++) {
        Key key = cube_distribution(random_engine);
        Value value = j;
        bool hasVal = test_map.find(key) != test_map.end();
        bool hasValRef = reference_map.find(key) != reference_map.end();
        assert(hasVal == hasValRef);
        reference_map.emplace(key, value);
        test_map.try_emplace(key, value);
        reverse_map.emplace_back(value, key);
    }
}

struct Id {
    Id() : first{-1}, _i{0} {
        ++default_construct_count_;
    }

    explicit Id(double dist, const size_t i) : first{dist}, _i{static_cast<int>(i)} {
        ++construct_count_;
    }

    explicit Id(double dist, const int i) : first{dist}, _i{i} {
        ++construct_count_;
    }

    Id(const Id& other) {
        ++copy_construct_count_;
        first = other.first;
        _i = other._i;
    }

    Id(Id&& other) noexcept {
        ++move_construct_count_;
        first = other.first;
        _i = other._i;
    }

    Id& operator=(const Id& other) noexcept {
        ++copy_assign_count_;
        first = other.first;
        _i = other._i;
        return *this;
    }
    Id& operator=(Id&& other) noexcept {
        ++move_assign_count_;
        first = other.first;
        _i = other._i;
        return *this;
    }

    bool operator==(const Id& rhs) const {
        return _i == rhs._i && first == rhs.first;
    }

    ~Id() {
        ++destruct_count_;
    }

    double first;
    int _i;
};

struct IdComparator {
    bool operator()(const Id& left, const Id& right) const {
        return left.first > right.first;
    }
};

template <typename Compare>
struct SwapComp {
    Compare comp;
    template <class Key>
    bool operator()(const Key& x, const Key& y) const {
        return !comp(x, y);
    }
};

void SmokeTest() {
    const size_t N = 1000;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N / 2);
    std::uniform_real_distribution<double> dist_distribution(0, 100);

    using ValueT = Id;
    for (int i = 0; i < 100; i++) {
        detail::priority_queue<ValueT, IdComparator> test_map;
        std::priority_queue<ValueT, std::vector<ValueT>, IdComparator> ref_heap{};

        // populate
        for (size_t j = 0; j < N; j++) {
            double dist = ((i + 1) * (j % 2)) == 0 ? 0 : dist_distribution(random_engine);
            ValueT val{dist, j};
            test_map.emplace(val);
            // test_map._check();
            ref_heap.emplace(val);

            ASSERT_EQ(test_map.size(), ref_heap.size());
            ASSERT_EQ(test_map.size(), j + 1u);

            ASSERT_EQ(test_map.top().first, ref_heap.top().first);
        }

        // update
        for (size_t j = 0; j < N; j++) {
            // pop
            double d1 = test_map.top().first;
            test_map.pop();
            ref_heap.pop();
            ASSERT_EQ(test_map.top().first, ref_heap.top().first);
            ASSERT_LE(d1, test_map.top().first);

            // push
            double dist = (i * (j % 2)) == 0 ? 0 : dist_distribution(random_engine);
            ValueT val{dist, j};
            test_map.emplace(val);
            ref_heap.emplace(val);

            ASSERT_EQ(test_map.size(), ref_heap.size());
            ASSERT_EQ(test_map.size(), N);

            ASSERT_EQ(test_map.top().first, ref_heap.top().first);
        }

        // drain
        double prev_dist = 0;
        for (size_t j = 0; j < N; j++) {
            ASSERT_EQ(test_map.top().first, ref_heap.top().first);
            ASSERT_LE(prev_dist, test_map.top().first);
            double dist = test_map.top().first;
            test_map.pop();
            ref_heap.pop();
            prev_dist = dist;
        }

        ASSERT_EQ(0u, test_map.size());
        ASSERT_TRUE(test_map.empty());
    }
}

TEST(PhTreeBptPriorityQueueTest, SmokeTest) {
    SmokeTest();
}

void SmokeTestTop() {
    const size_t N = 1000;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N / 2);
    std::uniform_real_distribution<double> dist_distribution(0, 100);

    using ValueT = Id;
    for (int i = 0; i < 100; i++) {
        detail::priority_queue<ValueT, IdComparator> test_map;
        std::priority_queue<ValueT, std::vector<ValueT>, SwapComp<IdComparator>> ref_heap_inverse{};

        // populate
        for (size_t j = 0; j < N; j++) {
            double dist = ((i + 1) * (j % 2)) == 0 ? 0 : dist_distribution(random_engine);
            ValueT val{dist, j};
            test_map.emplace(val);
            // test_map._check();
            ref_heap_inverse.emplace(val);

            ASSERT_EQ(test_map.size(), ref_heap_inverse.size());
            ASSERT_EQ(test_map.size(), j + 1u);

            ASSERT_EQ(test_map.top_max().first, ref_heap_inverse.top().first);
        }

        // update
        for (size_t j = 0; j < N; j++) {
            // pop
            double d1 = test_map.top().first;
            test_map.pop_max();
            ref_heap_inverse.pop();
            ASSERT_EQ(test_map.top_max().first, ref_heap_inverse.top().first);
            ASSERT_LE(d1, test_map.top().first);

            // push
            double dist = (i * (j % 2)) == 0 ? 0 : dist_distribution(random_engine);
            ValueT val{dist, j};
            test_map.emplace(val);
            ref_heap_inverse.emplace(val);

            ASSERT_EQ(test_map.size(), ref_heap_inverse.size());
            ASSERT_EQ(test_map.size(), N);

            ASSERT_EQ(test_map.top_max().first, ref_heap_inverse.top().first);
        }

        // drain
        double prev_dist = 0;
        for (size_t j = 0; j < N; j++) {
            ASSERT_EQ(test_map.top_max().first, ref_heap_inverse.top().first);
            ASSERT_LE(prev_dist, test_map.top().first);
            double dist = test_map.top().first;
            test_map.pop_max();
            ref_heap_inverse.pop();
            prev_dist = dist;
        }

        ASSERT_EQ(0u, test_map.size());
        ASSERT_TRUE(test_map.empty());
    }
}

TEST(PhTreeBptPriorityQueueTest, SmokeTestTop) {
    SmokeTestTop();
}

TEST(PhTreeBptPriorityQueueTest, DestructionTest) {
    const size_t N = 1000;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N / 2);
    std::uniform_real_distribution<double> dist_distribution(0, 100);

    using ValueT = Id;
    for (int i = 0; i < 10; i++) {
        detail::priority_queue<ValueT, IdComparator> test_map;

        // populate
        for (size_t j = 0; j < N; j++) {
            double dist = ((i + 1) * (j % 2)) == 0 ? 0 : dist_distribution(random_engine);
            ValueT val{dist, j};
            test_map.emplace(val);
        }
        // Remove some (or not)
        for (size_t j = 0; j < i * N / 100; j++) {
            double dist = ((i + 1) * (j % 2)) == 0 ? 0 : dist_distribution(random_engine);
            ValueT val{dist, j};
            test_map.pop();
            test_map.pop_max();
        }
        // Automatic destruction happens here.
    }
}

template <typename TREE>
void test_tree(TREE& tree) {
    // test various operations
    tree.emplace(Id(43, 2));
    Id id3{44, 3};
    tree.emplace(id3);
    ASSERT_EQ(tree.size(), 3u);

    tree.pop();
    ASSERT_EQ(2u, tree.size());
    tree.pop();
    tree.pop();
    ASSERT_EQ(0u, tree.size());
    ASSERT_TRUE(tree.empty());
}

TEST(PhTreeBptPriorityQueueTest, TestCopyConstruct) {
    using TestTree = detail::priority_queue<Id, IdComparator>;
    TestTree tree1;
    tree1.emplace(Id(42, 1));

    TestTree tree{tree1};
    test_tree(tree);
    // The old tree should still work!
    test_tree(tree1);
}

TEST(PhTreeBptPriorityQueueTest, TestCopyAssign) {
    using TestTree = detail::priority_queue<Id, IdComparator>;
    TestTree tree1;
    tree1.emplace(Id(42, 1));

    TestTree tree{};
    tree = tree1;
    test_tree(tree);
    // The old tree should still work!
    test_tree(tree1);
}

TEST(PhTreeBptPriorityQueueTest, TestMoveConstruct) {
    using TestTree = detail::priority_queue<Id, IdComparator>;
    TestTree tree1;
    tree1.emplace(Id(42, 1));

    TestTree tree{std::move(tree1)};
    test_tree(tree);
}

TEST(PhTreeBptPriorityQueueTest, TestMoveAssign) {
    using TestTree = detail::priority_queue<Id, IdComparator>;
    TestTree tree1;
    tree1.emplace(Id(42, 1));

    TestTree tree{};
    tree = std::move(tree1);
    test_tree(tree);
}
