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

#include "phtree/common/b_plus_tree_multimap.h"
#include <include/gtest/gtest.h>
#include <random>
#include <unordered_map>

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
    b_plus_tree_multimap<Key, Value>& test_map,
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
    Id() : _i{0} {
        ++default_construct_count_;
    }

    explicit Id(const size_t i) : _i{static_cast<int>(i)} {
        ++construct_count_;
    }

    explicit Id(const int i) : _i{i} {
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

template <typename R, typename K, typename V, typename END>
void CheckMapResult(const R& result, END end, const K& key, const V& val) {
    ASSERT_NE(result, end);
    ASSERT_EQ(result->first, key);
    ASSERT_EQ(result->second, val);
}

void SmokeTestMap() {
    const int N = 300;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N / 2);

    using Key = size_t;
    using Value = Id;
    for (int i = 0; i < 10; i++) {
        b_plus_tree_multimap<Key, Value> test_map;
        std::unordered_multimap<Key, Value> reference_map{};
        std::vector<std::pair<Value, Key>> reverse_map;
        for (int j = 0; j < N; j++) {
            size_t key = cube_distribution(random_engine);
            bool hasVal = test_map.find(key) != test_map.end();
            bool hasValRef = reference_map.find(key) != reference_map.end();
            ASSERT_EQ(hasVal, hasValRef);

            Value val{j};
            if (key % 6 == 0) {
                CheckMapResult(test_map.emplace(key, val), test_map.end(), key, val);
            } else if (key % 6 == 1) {
                CheckMapResult(test_map.try_emplace(key, val), test_map.end(), key, val);
            } else if (key % 6 == 2) {
                // Leaf-hint of questionable quality
                auto hint = test_map.find(key - 1);
                CheckMapResult(test_map.try_emplace(hint, key, val), test_map.end(), key, val);
            } else if (key % 6 == 3) {
                auto hint = j % 2 == 0 ? test_map.begin() : test_map.end();
                // Bad hint
                CheckMapResult(test_map.try_emplace(hint, key, val), test_map.end(), key, val);
            } else if (key % 6 == 4) {
                // Leaf-hint of questionable quality
                auto hint = test_map.find(key - 1);
                CheckMapResult(test_map.emplace_hint(hint, key, val), test_map.end(), key, val);
            } else {
                auto hint = j % 2 == 0 ? test_map.begin() : test_map.end();
                // Bad hint
                CheckMapResult(test_map.emplace_hint(hint, key, val), test_map.end(), key, val);
            }
            test_map._check();
            reference_map.emplace(key, val);
            reverse_map.emplace_back(val, key);

            ASSERT_EQ(test_map.size(), reference_map.size());
            ASSERT_EQ(test_map.size(), j + 1u);

            Key prev_key = 0;
            for (auto& entry : test_map) {
                auto& eMap = *test_map.find(entry.first);
                ASSERT_EQ(entry.first, eMap.first);
                ASSERT_LE(prev_key, eMap.first);
                prev_key = eMap.first;
                auto& eRef = reverse_map[eMap.second._i];
                ASSERT_EQ(eMap.second, eRef.first);
                ASSERT_EQ(eMap.first, eRef.second);
            }
        }
    }
}

TEST(PhTreeBptMulitmapTest, SmokeTestNonUnique) {
    SmokeTestMap();
}

TEST(PhTreeBptMulitmapTest, SmokeTestWithTryEmplace) {
    const int N = 200;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N / 2);

    using Key = size_t;
    using Val = size_t;
    for (int i = 0; i < 10; i++) {
        b_plus_tree_multimap<Key, Val> test_map;
        std::map<Key, Val> reference_map;
        for (int j = 0; j < N; j++) {
            Key key = cube_distribution(random_engine);
            Val val = key;
            bool hasVal = test_map.find(key) != test_map.end();
            bool hasValRef = reference_map.find(key) != reference_map.end();
            ASSERT_EQ(hasVal, hasValRef);
            if (!hasVal) {
                reference_map.emplace(key, val);
                test_map.try_emplace(key, val);
            }
            ASSERT_EQ(test_map.size(), reference_map.size());
            for (auto entry : reference_map) {
                size_t vRef = entry.first;
                size_t vMap = test_map.find(vRef)->second;
                ASSERT_EQ(vMap, vRef);
            }
            for (auto entry : test_map) {
                size_t v = entry.first;
                size_t vRef = reference_map.find(v)->second;
                size_t vMap = test_map.find(v)->second;
                ASSERT_EQ(vMap, vRef);
            }
        }
    }
}

TEST(PhTreeBptMulitmapTest, SmokeTestWithEraseByKey) {
    const int N = 200;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N / 2);

    using Key = size_t;
    using Value = size_t;
    for (int i = 0; i < 10; i++) {
        b_plus_tree_multimap<Key, Value> test_map{};
        std::multimap<Key, Value> reference_map{};
        std::vector<std::pair<Value, Key>> reverse_map{};
        populate(N, test_map, reference_map, reverse_map, random_engine);

        std::shuffle(reverse_map.begin(), reverse_map.end(), random_engine);
        // We iterator over all entries even though every erase() may remove several entries.
        // -> This also tests behavior for non-existing keys (that have already been removed).
        for (auto reverse_pair : reverse_map) {
            auto key = reverse_pair.second;

            auto result_test = test_map.erase(key);
            auto result_ref = reference_map.erase(key);
            assert(result_test == result_ref);
            ASSERT_EQ(result_test, result_ref);

            test_map._check();
            ASSERT_EQ(test_map.size(), reference_map.size());
            for (auto& entry : reference_map) {
                const Key& vRef = entry.first;
                Key vMap = test_map.find(vRef)->first;
                ASSERT_EQ(vMap, vRef);
            }
            for (auto& entry : test_map) {
                Key v = entry.first;
                const Key& vRef = reference_map.find(v)->first;
                Key vMap = test_map.find(v)->first;
                ASSERT_EQ(vMap, vRef);
            }
        }
    }
}

TEST(PhTreeBptMulitmapTest, SmokeTestWithEraseByIterator) {
    const int N = 200;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N / 2);

    using Key = size_t;
    using Value = size_t;
    for (int i = 0; i < 10; i++) {
        b_plus_tree_multimap<Key, Value> test_map{};
        std::multimap<Key, Value> reference_map{};
        std::vector<std::pair<Value, Key>> reverse_map{};
        populate(N, test_map, reference_map, reverse_map, random_engine);

        std::shuffle(reverse_map.begin(), reverse_map.end(), random_engine);
        for (auto reverse_pair : reverse_map) {
            auto key = reverse_pair.second;
            auto val = reverse_pair.first;

            auto it = test_map.find(key);
            while (it->second != val) {
                ++it;
            }
            auto next = it;
            ++next;
            auto is_last = next == test_map.end();
            auto next_val = is_last ? -1 : next->first;
            auto result = test_map.erase(it);
            if (is_last) {
                ASSERT_EQ(test_map.end(), result);
            } else {
                ASSERT_NE(test_map.end(), result);
                ASSERT_EQ(next_val, result->first);
            }
            auto ref_iter = reference_map.find(key);
            while (ref_iter != reference_map.end() && ref_iter->second != val) {
                ++ref_iter;
            }
            reference_map.erase(ref_iter);

            test_map._check();
            for (auto& entry : reference_map) {
                const Key& vRef = entry.first;
                Key vMap = test_map.find(vRef)->first;
                ASSERT_EQ(vMap, vRef);
            }
            for (auto& entry : test_map) {
                Key v = entry.first;
                const Key& vRef = reference_map.find(v)->first;
                Key vMap = test_map.find(v)->first;
                ASSERT_EQ(vMap, vRef);
            }
            ASSERT_EQ(test_map.size(), reference_map.size());
        }
    }
}

void SmokeTestWithErase(bool use_begin, bool use_end) {
    const int N = 300;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N / 2);
    std::uniform_real_distribution<> real_distribution(0, 1);

    using Key = size_t;
    using Value = size_t;
    for (int i = 0; i < 500; i++) {
        b_plus_tree_multimap<Key, Value> test_map{};
        std::multimap<Key, Value> reference_map{};
        std::vector<std::pair<Value, Key>> key_list{};
        populate(N, test_map, reference_map, key_list, random_engine);

        // Pick some random keys
        auto key1 = std::min(key_list[0].second, key_list[1].second);
        auto key2 = std::max(key_list[0].second, key_list[1].second);

        auto it_test_1 = use_begin ? test_map.begin() : test_map.find(key1);
        auto it_test_2 = use_end ? test_map.end() : test_map.find(key2);
        auto it_ref_1 = use_begin ? reference_map.begin() : reference_map.find(key1);
        auto it_ref_2 = use_end ? reference_map.end() : reference_map.find(key2);
        auto result_test = test_map.erase(it_test_1, it_test_2);
        auto result_ref = reference_map.erase(it_ref_1, it_ref_2);

        if (result_ref != reference_map.end()) {
            ASSERT_EQ(result_test->first, result_ref->first);
        } else {
            ASSERT_EQ(result_test, test_map.end());
        }

        // check len:
        size_t n = 0;
        for (auto& e : test_map) {
            (void)e;
            ++n;
        }
        ASSERT_EQ(n, test_map.size());

        test_map._check();
        auto it_test = test_map.begin();
        auto it_ref = reference_map.begin();
        while (it_test != test_map.end()) {
            ASSERT_NE(it_ref, reference_map.end());
            auto& r = *it_ref;
            auto& e = *it_test;
            ASSERT_EQ(e.first, r.first);
            // ASSERT_EQ(e.second, r.second); std::multi_map is inserttion ordered, b_p_t is not.
            ++it_test;
            ++it_ref;
        }
        ASSERT_EQ(it_test, test_map.end());
        ASSERT_EQ(it_ref, reference_map.end());
    }
}

TEST(PhTreeBptMulitmapTest, SmokeTestWithEraseInterval) {
    SmokeTestWithErase(false, false);
    SmokeTestWithErase(false, true);
    SmokeTestWithErase(true, false);
    SmokeTestWithErase(true, true);
}

TEST(PhTreeBptMulitmapTest, SmokeTestUpdateByIterator) {
    // This tests repeated erase()/insert()
    const int N = 200;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N / 20);

    using Key = size_t;
    using Value = size_t;
    b_plus_tree_multimap<Key, Value> test_map{};
    std::multimap<Key, Value> reference_map{};
    std::vector<std::pair<Value, Key>> reverse_map{};
    populate(N, test_map, reference_map, reverse_map, random_engine);
    for (int i = 0; i < 100; i++) {
        std::shuffle(reverse_map.begin(), reverse_map.end(), random_engine);
        for (auto& reverse_pair : reverse_map) {
            auto key = reverse_pair.second;
            auto val = reverse_pair.first;

            // reference map
            auto ref_iter = reference_map.find(key);
            while (ref_iter != reference_map.end() && ref_iter->second != val) {
                ++ref_iter;
            }
            ASSERT_NE(ref_iter, reference_map.end());
            reference_map.erase(ref_iter);

            // tested map
            auto it = test_map.find(key);
            ASSERT_NE(it, test_map.end());
            while (it->second != val) {
                ++it;
            }
            auto next = it;
            // verify return value
            ++next;
            auto is_last = next == test_map.end();
            auto next_val = is_last ? -1 : next->first;
            auto result = test_map.erase(it);
            if (is_last) {
                ASSERT_EQ(test_map.end(), result);
            } else {
                ASSERT_NE(test_map.end(), result);
                ASSERT_EQ(next_val, result->first);
            }

            test_map._check();

            // insert again
            reverse_pair.second = cube_distribution(random_engine);
            test_map.emplace(reverse_pair.second, reverse_pair.first);
            reference_map.emplace(reverse_pair.second, reverse_pair.first);

            test_map._check();

            for (auto& entry : reference_map) {
                const Key& vRef = entry.first;
                Key vMap = test_map.find(vRef)->first;
                ASSERT_EQ(vMap, vRef);
            }
            for (auto& entry : test_map) {
                Key v = entry.first;
                const Key& vRef = reference_map.find(v)->first;
                Key vMap = test_map.find(v)->first;
                ASSERT_EQ(vMap, vRef);
            }
            ASSERT_EQ(test_map.size(), reference_map.size());
        }
    }
}

template <typename TREE>
void test_tree(TREE& tree) {
    using Key = size_t;
    using Value = size_t;
    Key p{42};

    // test various operations
    tree.emplace(p, Value{2});
    Value id3{3};
    tree.emplace(p, id3);
    ASSERT_EQ(tree.size(), 3u);

    auto q_extent = tree.begin();
    ASSERT_NE(q_extent, tree.end());
    ++q_extent;
    ASSERT_NE(q_extent, tree.end());
    ++q_extent;
    ASSERT_NE(q_extent, tree.end());
    ++q_extent;
    ASSERT_EQ(q_extent, tree.end());

    ASSERT_EQ(3u, tree.erase(p));
    ASSERT_EQ(0u, tree.size());
    ASSERT_EQ(0u, tree.erase(p));
    ASSERT_EQ(0u, tree.size());
    ASSERT_TRUE(tree.empty());
}

TEST(PhTreeBptMulitmapTest, TestCopyConstruct) {
    using TestTree = b_plus_tree_multimap<size_t, size_t>;
    TestTree tree1;
    tree1.emplace(42, 1);

    TestTree tree{tree1};
    test_tree(tree);
    // The old tree should still work!
    test_tree(tree1);
}

TEST(PhTreeBptMulitmapTest, TestCopyAssign) {
    using TestTree = b_plus_tree_multimap<size_t, size_t>;
    TestTree tree1;
    tree1.emplace(42, 1);

    TestTree tree{};
    tree = tree1;
    test_tree(tree);
    // The old tree should still work!
    test_tree(tree1);
}

TEST(PhTreeBptMulitmapTest, TestMoveConstruct) {
    using TestTree = b_plus_tree_multimap<size_t, size_t>;
    TestTree tree1;
    tree1.emplace(42, 1);

    TestTree tree{std::move(tree1)};
    test_tree(tree);
}

TEST(PhTreeBptMulitmapTest, TestMoveAssign) {
    using TestTree = b_plus_tree_multimap<size_t, size_t>;
    TestTree tree1;
    tree1.emplace(42, 1);

    TestTree tree{};
    tree = std::move(tree1);
    test_tree(tree);
}

TEST(PhTreeBptMulitmapTest, TestMovableIterators) {
    using Key = size_t;
    using Value = size_t;
    using TestTree = b_plus_tree_multimap<Key, Value>;
    // Test edge case: only one entry in tree
    Key p{42};
    auto tree = TestTree();
    tree.emplace(p, Value{1});

    ASSERT_TRUE(std::is_move_constructible_v<decltype(tree.begin())>);
    ASSERT_TRUE(std::is_move_assignable_v<decltype(tree.begin())>);
    ASSERT_NE(tree.begin(), tree.end());

    ASSERT_TRUE(std::is_move_constructible_v<decltype(tree.end())>);
    ASSERT_TRUE(std::is_move_assignable_v<decltype(tree.end())>);

    ASSERT_TRUE(std::is_move_constructible_v<decltype(tree.find(p))>);
    ASSERT_TRUE(std::is_move_assignable_v<decltype(tree.find(p))>);
    ASSERT_NE(tree.find(p), tree.end());
}
