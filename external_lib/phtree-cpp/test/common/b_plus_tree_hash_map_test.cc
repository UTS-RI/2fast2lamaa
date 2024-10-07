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

#include "phtree/common/b_plus_tree_hash_map.h"
#include <include/gtest/gtest.h>
#include <random>
#include <unordered_map>
#include <unordered_set>

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

namespace std {
template <>
struct hash<Id> {
    size_t operator()(const Id& x) const {
        return std::hash<int>{}(x._i % 10);
    }
};
};  // namespace std

template <typename R, typename K, typename V, typename END>
void CheckMapResult(const R& result, END end, const K& key, const V& val) {
    ASSERT_NE(result, end);
    ASSERT_EQ(result->first, key);
    ASSERT_EQ(result->second, val);
}

template <typename R, typename K, typename V>
void CheckMapResultPair(const R& result, bool expected_success, const K& key, const V& val) {
    assert(result.second == expected_success);
    ASSERT_EQ(result.second, expected_success);
    ASSERT_EQ(result.first->first, key);
    ASSERT_EQ(result.first->second, val);
}

template <typename R, typename K, typename END>
void CheckSetResult(const R& result, END end, const K& key) {
    ASSERT_NE(result, end);
    ASSERT_EQ(*result, key);
}

template <typename R, typename K>
void CheckSetResultPair(const R& result, bool expected_success, const K& key) {
    assert(result.second == expected_success);
    ASSERT_EQ(result.second, expected_success);
    ASSERT_EQ(*result.first, key);
}

template <typename HashT>
void SmokeTestMap() {
    const int N = 300;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N / 2);

    size_t val = 0;
    for (int i = 0; i < 10; i++) {
        b_plus_tree_hash_map<Id, size_t, HashT, std::equal_to<>> test_map;
        std::unordered_map<Id, size_t> reference_map;
        for (int j = 0; j < N; j++) {
            size_t key = cube_distribution(random_engine);
            Id id(key);
            bool hasVal = test_map.find(id) != test_map.end();
            bool hasValRef = reference_map.find(id) != reference_map.end();
            ASSERT_EQ(hasVal, hasValRef);
            auto iter_lb = test_map.lower_bound(id);
            bool hasValLB = iter_lb != test_map.end() && iter_lb->first == id;
            ASSERT_EQ(hasVal, hasValLB);

            if (!hasVal) {
                if (key % 6 == 0) {
                    CheckMapResultPair(test_map.emplace(id, val), true, id, val);
                    CheckMapResultPair(test_map.emplace(id, val), false, id, val);
                } else if (key % 6 == 1) {
                    CheckMapResultPair(test_map.try_emplace(id, val), true, id, val);
                    CheckMapResultPair(test_map.try_emplace(id, val), false, id, val);
                } else if (key % 6 == 2) {
                    // Leaf-hint of questionable quality
                    auto hint = test_map.find(Id(key - 1));
                    CheckMapResult(test_map.try_emplace(hint, id, val), test_map.end(), id, val);
                    CheckMapResult(test_map.try_emplace(hint, id, val), test_map.end(), id, val);
                } else if (key % 6 == 3) {
                    auto hint = j % 2 == 0 ? test_map.begin() : test_map.end();
                    // Bad hint
                    CheckMapResult(test_map.try_emplace(hint, id, val), test_map.end(), id, val);
                    CheckMapResult(test_map.try_emplace(hint, id, val), test_map.end(), id, val);
                } else if (key % 6 == 4) {
                    // Leaf-hint of questionable quality
                    auto hint = test_map.find(Id(key - 1));
                    CheckMapResult(test_map.emplace_hint(hint, id, val), test_map.end(), id, val);
                    CheckMapResult(test_map.emplace_hint(hint, id, val), test_map.end(), id, val);
                } else {
                    auto hint = j % 2 == 0 ? test_map.begin() : test_map.end();
                    // Bad hint
                    CheckMapResult(test_map.emplace_hint(hint, id, val), test_map.end(), id, val);
                    CheckMapResult(test_map.emplace_hint(hint, id, val), test_map.end(), id, val);
                }
                test_map._check();
                reference_map.emplace(id, val);
            }

            ASSERT_EQ(test_map.size(), reference_map.size());
            for (auto& entry : reference_map) {
                const Id& kRef = entry.first;
                size_t vMap = test_map.find(kRef)->second;
                ASSERT_EQ(vMap, entry.second);
                ASSERT_TRUE(test_map.count(kRef));
            }
            for (auto& entry : test_map) {
                Id& k = entry.first;
                size_t vRef = reference_map.find(k)->second;
                size_t vMap = test_map.find(k)->second;
                ASSERT_EQ(vMap, vRef);
            }
            ++val;
        }
    }
}

TEST(PhTreeBptHashMapTest, SmokeTestNonUnique) {
    SmokeTestMap<std::hash<Id>>();
}

TEST(PhTreeBptHashMapTest, SmokeTestSameHash) {
    struct DumbHash {
        size_t operator()(const Id&) const {
            return 42;
        }
    };
    SmokeTestMap<DumbHash>();
}

template <typename Hash>
void SmokeTestSet() {
    const int N = 200;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N / 2);

    for (int i = 0; i < 10; i++) {
        b_plus_tree_hash_set<Id, Hash> test_map;
        std::unordered_set<Id> reference_map;
        for (int j = 0; j < N; j++) {
            {
                size_t key = cube_distribution(random_engine);
                Id id(key);
                bool hasVal = test_map.find(id) != test_map.end();
                bool hasValRef = reference_map.find(id) != reference_map.end();
                ASSERT_EQ(hasVal, hasValRef);
                auto iter_lb = test_map.lower_bound(id);
                bool hasValLB = iter_lb != test_map.end() && *iter_lb == id;
                ASSERT_EQ(hasVal, hasValLB);

                if (!hasVal) {
                    if (key % 3 == 0) {
                        CheckSetResultPair(test_map.emplace(id), true, id);
                        CheckSetResultPair(test_map.emplace(key), false, id);
                    } else if (key % 3 == 1) {
                        // Leaf-hint of questionable quality
                        auto hint = test_map.find(Id(key - 1));
                        CheckSetResult(test_map.emplace_hint(hint, id), test_map.end(), id);
                        CheckSetResult(test_map.emplace_hint(hint, key), test_map.end(), id);
                    } else {
                        auto hint = j % 2 == 0 ? test_map.begin() : test_map.end();
                        // Bad hint
                        CheckSetResult(test_map.emplace_hint(hint, id), test_map.end(), id);
                        CheckSetResult(test_map.emplace_hint(hint, key), test_map.end(), id);
                    }
                    test_map._check();
                    reference_map.emplace(id);
                }
            }

            ASSERT_EQ(test_map.size(), reference_map.size());
            for (auto& id : reference_map) {
                Id& idMap = *test_map.find(id);
                ASSERT_EQ(idMap, id);
            }
            for (auto& id : test_map) {
                const Id& vRef = *reference_map.find(id);
                Id& vMap = *test_map.find(id);
                ASSERT_EQ(vMap, vRef);
            }
        }
    }
}

TEST(PhTreeBptHashSetTest, SmokeTestNonUnique) {
    SmokeTestSet<std::hash<Id>>();
}

TEST(PhTreeBptHashSetTest, SmokeTestSameHash) {
    struct DumbHash {
        size_t operator()(const Id&) const {
            return 42;
        }
    };
    SmokeTestSet<DumbHash>();
}

TEST(PhTreeBptHashMapTest, SmokeTestWithTryEmplace) {
    const int N = 200;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N / 2);

    for (int i = 0; i < 10; i++) {
        b_plus_tree_hash_map<size_t, size_t, std::hash<size_t>, std::equal_to<>> test_map;
        std::map<size_t, size_t> reference_map;
        for (int j = 0; j < N; j++) {
            size_t val = cube_distribution(random_engine);
            bool hasVal = test_map.find(val) != test_map.end();
            bool hasValRef = reference_map.find(val) != reference_map.end();
            ASSERT_EQ(hasVal, hasValRef);
            if (!hasVal) {
                reference_map.emplace(val, val);
                test_map.try_emplace(val, val);
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

template <typename HashT>
void SmokeTestWithErase(bool by_iterator) {
    const int N = 200;
    std::default_random_engine random_engine{0};
    std::uniform_int_distribution<> cube_distribution(0, N / 2);

    for (int i = 0; i < 10; i++) {
        b_plus_tree_hash_map<Id, size_t, HashT, std::equal_to<>> test_map{};
        std::unordered_map<Id, size_t> reference_map{};
        std::vector<size_t> key_list{};
        for (int j = 0; j < N; j++) {
            size_t key = cube_distribution(random_engine);
            Id id(key);
            bool hasVal = test_map.find(id) != test_map.end();
            bool hasValRef = reference_map.find(id) != reference_map.end();
            ASSERT_EQ(hasVal, hasValRef);
            reference_map.emplace(id, key);
            test_map.try_emplace(id, key);
            key_list.emplace_back(key);
        }

        int x = 0;
        std::shuffle(key_list.begin(), key_list.end(), random_engine);
        for (auto key : key_list) {
            Id id(key);
            // This may try to erase an entry that does not exist!
            auto it = test_map.find(id);
            if (it == test_map.end()) {
                ASSERT_EQ(0u, reference_map.erase(id));
                continue;
            }
            if (by_iterator) {
                auto next = it;
                ++next;
                auto is_last = next == test_map.end();
                auto next_val = is_last ? Id(-1) : next->first;
                auto result = test_map.erase(it);
                if (is_last) {
                    ASSERT_EQ(test_map.end(), result);
                } else {
                    ASSERT_NE(test_map.end(), result);
                    ASSERT_EQ(next_val, result->first);
                }
            } else {
                test_map.erase(id);
            }
            test_map._check();
            ASSERT_EQ(1u, reference_map.erase(id));
            for (auto& entry : reference_map) {
                const Id& vRef = entry.first;
                Id& vMap = test_map.find(vRef)->first;
                ASSERT_EQ(vMap, vRef);
            }
            for (auto& entry : test_map) {
                Id& v = entry.first;
                const Id& vRef = reference_map.find(v)->first;
                Id& vMap = test_map.find(v)->first;
                ASSERT_EQ(vMap, vRef);
            }
            ASSERT_EQ(test_map.size(), reference_map.size());
            ++x;
        }
    }
}

TEST(PhTreeBptHashMapTest, SmokeTestWithErase) {
    SmokeTestWithErase<std::hash<Id>>(true);
    SmokeTestWithErase<std::hash<Id>>(false);
}

TEST(PhTreeBptHashMapTest, SmokeTestWithEraseSameHash) {
    struct DumbHash {
        size_t operator()(const Id&) const {
            return 42;
        }
    };
    SmokeTestWithErase<DumbHash>(true);
    SmokeTestWithErase<DumbHash>(false);
}

template <typename TREE>
void test_tree(TREE& tree, size_t size = 1) {
    using Key = size_t;
    using Value = size_t;
    Key p{43};

    // test various operations
    tree.emplace(p, Value{2});
    Value id3{3};
    tree.emplace(p, id3);
    ASSERT_EQ(tree.size(), 1u + size);

    auto q_extent = tree.begin();
    ASSERT_NE(q_extent, tree.end());
    size_t n = 0;
    while (q_extent != tree.end()) {
        ++q_extent;
        ++n;
    }
    ASSERT_EQ(q_extent, tree.end());
    ASSERT_EQ(size + 1u, n);

    tree.erase(p);
    ASSERT_EQ(0u + size, tree.size());
    tree.erase(p);
    ASSERT_EQ(0u + size, tree.size());
}

// TEST(PhTreeBptHashMapTest, TestCopyConstruct) {
//     using TestTree = b_plus_tree_hash_map<size_t, size_t>;
//     TestTree tree1;
//     tree1.emplace(42, 1);
//
//     TestTree tree{tree1};
//     test_tree(tree);
//     // The old tree should still work!
//     test_tree(tree1);
// }
//
// TEST(PhTreeBptHashMapTest, TestCopyAssign) {
//     using TestTree = b_plus_tree_hash_map<size_t, size_t>;
//     TestTree tree1;
//     tree1.emplace(42, 1);
//
//     TestTree tree{};
//     tree = tree1;
//     test_tree(tree);
//     // The old tree should still work!
//     test_tree(tree1);
// }

TEST(PhTreeBptHashMapTest, TestMoveConstruct) {
    using TestTree = b_plus_tree_hash_map<size_t, size_t>;
    TestTree tree1;
    tree1.emplace(42, 1);

    TestTree tree{std::move(tree1)};
    test_tree(tree);
}

TEST(PhTreeBptHashMapTest, TestMoveAssign) {
    using TestTree = b_plus_tree_hash_map<size_t, size_t>;
    TestTree tree1;
    tree1.emplace(42, 1);

    TestTree tree{};
    tree = std::move(tree1);
    test_tree(tree);
}

// TEST(PhTreeBptHashMapTest, TestCopyConstructLarge) {
//     using TestTree = b_plus_tree_hash_map<size_t, size_t>;
//     TestTree tree1;
//     for (int i = 0; i < 100; ++i) {
//         tree1.emplace(100 + i, i);
//     }
//
//     TestTree tree{tree1};
//     test_tree(tree, 100);
//     // The old tree should still work!
//     test_tree(tree1, 100);
// }
//
// TEST(PhTreeBptHashMapTest, TestCopyAssignLarge) {
//     using TestTree = b_plus_tree_hash_map<size_t, size_t>;
//     TestTree tree1;
//     for (int i = 0; i < 100; ++i) {
//         tree1.emplace(100 + i, i);
//     }
//
//     TestTree tree{};
//     tree = tree1;
//     test_tree(tree, 100);
//     // The old tree should still work!
//     test_tree(tree1, 100);
// }

TEST(PhTreeBptHashMapTest, TestMovableIterators) {
    using Key = size_t;
    using Value = size_t;
    using TestTree = b_plus_tree_hash_map<Key, Value>;
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
