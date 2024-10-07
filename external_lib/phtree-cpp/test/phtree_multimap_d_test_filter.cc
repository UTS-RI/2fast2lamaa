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

namespace phtree_multimap_d_test_filter {

// Number of entries that have the same coordinate
static const size_t NUM_DUPL = 4;
[[maybe_unused]] static const double WORLD_MIN = -1000;
[[maybe_unused]] static const double WORLD_MAX = 1000;

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

struct Id {
    Id() = default;

    explicit Id(const int i) : _i(i){};

    bool operator==(const Id& rhs) const {
        return _i == rhs._i;
    }

    Id(Id const& rhs) = default;
    Id(Id&& rhs) = default;
    Id& operator=(Id const& rhs) = default;
    Id& operator=(Id&& rhs) = default;

    int _i;
};
}  // namespace phtree_multimap_d_test_filter

namespace std {
template <>
struct hash<phtree_multimap_d_test_filter::Id> {
    size_t operator()(const phtree_multimap_d_test_filter::Id& x) const {
        return std::hash<int>{}(x._i);
    }
};
};  // namespace std

namespace phtree_multimap_d_test_filter {

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
        // create duplicates, ie. entries with the same coordinates. However, avoid unintentional
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

template <dimension_t DIM>
void populate(TestTree<DIM, size_t>& tree, std::vector<TestPoint<DIM>>& points, size_t N) {
    generateCube(points, N);
    for (size_t i = 0; i < N; i++) {
        ASSERT_TRUE(tree.insert(points[i], i).second);
    }
    ASSERT_EQ(N, tree.size());
}

static int f_default_construct_ = 0;
static int f_construct_ = 0;
static int f_copy_construct_ = 0;
static int f_move_construct_ = 0;
static int f_copy_assign_ = 0;
static int f_move_assign_ = 0;
static int f_destruct_ = 0;

static void f_reset_id_counters() {
    f_default_construct_ = 0;
    f_construct_ = 0;
    f_copy_construct_ = 0;
    f_move_construct_ = 0;
    f_copy_assign_ = 0;
    f_move_assign_ = 0;
    f_destruct_ = 0;
}

template <dimension_t DIM, typename T>
struct FilterCount {
    FilterCount() : last_known{} {
        ++f_default_construct_;
    }

    explicit FilterCount(const T i) : last_known{i} {
        ++f_construct_;
    }

    FilterCount(const FilterCount& other) {
        ++f_copy_construct_;
        last_known = other.last_known;
    }

    FilterCount(FilterCount&& other) noexcept {
        ++f_move_construct_;
        last_known = other.last_known;
    }

    FilterCount& operator=(const FilterCount& other) noexcept {
        ++f_copy_assign_;
        last_known = other.last_known;
        return *this;
    }
    FilterCount& operator=(FilterCount&& other) noexcept {
        ++f_move_assign_;
        last_known = other.last_known;
        return *this;
    }

    ~FilterCount() {
        ++f_destruct_;
    }

    template <typename BucketT>
    [[nodiscard]] constexpr bool IsEntryValid(const PhPoint<DIM>&, const BucketT& bucket) {
        assert(!bucket.empty());
        return true;
    }

    template <typename T2>
    [[nodiscard]] bool IsBucketEntryValid(const PhPoint<DIM>&, const T2& value) {
        last_known = value;
        return true;
    }

    [[nodiscard]] constexpr bool IsNodeValid(const PhPoint<DIM>&, int) {
        return true;
    }

    T last_known;
};

template <dimension_t DIM>
struct DistanceCount {
    DistanceCount() {
        ++f_default_construct_;
    }

    DistanceCount(const DistanceCount&) {
        ++f_copy_construct_;
    }

    DistanceCount(DistanceCount&&) noexcept {
        ++f_move_construct_;
    }

    DistanceCount& operator=(const DistanceCount&) noexcept {
        ++f_copy_assign_;
        return *this;
    }
    DistanceCount& operator=(DistanceCount&&) noexcept {
        ++f_move_assign_;
        return *this;
    }

    ~DistanceCount() {
        ++f_destruct_;
    }

    double operator()(const PhPointD<DIM>& p1, const PhPointD<DIM>& p2) const {
        double sum2 = 0;
        for (dimension_t i = 0; i < DIM; ++i) {
            double d2 = p1[i] - p2[i];
            sum2 += d2 * d2;
        }
        return sqrt(sum2);
    };
};

static size_t static_id = 0;

template <dimension_t DIM>
struct CallbackCount {
    CallbackCount() {
        static_id = 0;
        ++f_default_construct_;
    }

    CallbackCount(const CallbackCount&) {
        ++f_copy_construct_;
    }

    CallbackCount(CallbackCount&&) noexcept {
        ++f_move_construct_;
    }

    CallbackCount& operator=(const CallbackCount&) noexcept {
        ++f_copy_assign_;
        return *this;
    }
    CallbackCount& operator=(CallbackCount&&) noexcept {
        ++f_move_assign_;
        return *this;
    }

    ~CallbackCount() {
        ++f_destruct_;
    }

    void operator()(const TestPoint<DIM>, const Id& t) {
        static_id = t._i;
    }
};

template <dimension_t DIM, typename T>
struct FilterConst {
    template <typename BucketT>
    [[nodiscard]] constexpr bool IsEntryValid(const PhPoint<DIM>&, const BucketT&) const {
        return true;
    }
    [[nodiscard]] constexpr bool IsBucketEntryValid(const PhPoint<DIM>&, const T& value) {
        assert(value._i == 1);
        return true;
    }
    [[nodiscard]] constexpr bool IsNodeValid(const PhPoint<DIM>&, int) const {
        return true;
    }
};

template <dimension_t DIM>
struct CallbackConst {
    void operator()(const TestPoint<DIM>, const Id& t) const {
        static_id = t._i;
    }
};

[[maybe_unused]] static void print_id_counters() {
    std::cout << "dc=" << f_default_construct_ << " c=" << f_construct_
              << " cc=" << f_copy_construct_ << " mc=" << f_move_construct_
              << " ca=" << f_copy_assign_ << " ma=" << f_move_assign_ << " d=" << f_destruct_
              << std::endl;
}

/*
 * General comment: We are testing several thing here.
 * - If we pass lvalue filters/callbacks/... we want to ensure that they do not get copied or
 *   moved at all. We need to ensure that the lvalue argument is the same instance that is
 *   used internally by the iterator.
 * - If we pass a rvalue filters/callbacks/..., preventing copies/moves is harder. We are testing
 *   somewhat arbitrarily for a limit of 3 moves/copies per argument.
 * - We want to ensure that both rvalue/lvalue arguments work.
 * - We also do some limited testing that it works with 'const' trees.
 * - Finally, we test separately that the old legacy filters still work
 */

TEST(PhTreeTest, TestFilterAPI_FOR_EACH) {
    // Test edge case: only one entry in tree
    PhPointD<3> p{1, 2, 3};
    auto tree = TestTree<3, Id>();
    tree.emplace(p, Id{1});

    CallbackCount<3> callback;
    FilterCount<3, Id> filter{};
    // rvalue
    tree.for_each(callback, filter);
    ASSERT_EQ(static_id, 1);
    ASSERT_EQ(filter.last_known._i, 1);
    ASSERT_EQ(2, f_construct_ + f_default_construct_);
    ASSERT_EQ(0, f_copy_construct_ + f_move_construct_ + f_copy_assign_ + f_move_assign_);
    f_reset_id_counters();

    // lvalue
    tree.for_each(CallbackCount<3>(), FilterCount<3, Id>());
    ASSERT_EQ(static_id, 1);
    ASSERT_EQ(2, f_construct_ + f_default_construct_);
    ASSERT_GE(4, f_copy_construct_ + f_move_construct_ + f_copy_assign_ + f_move_assign_);
    f_reset_id_counters();

    // const Tree: just test that it compiles
    const TestTree<3, Id>& treeC = tree;
    // lvalue
    CallbackCount<3> callbackC;
    FilterConst<3, Id> filterC;
    treeC.for_each(callbackC, filterC);
    // rvalue
    treeC.for_each(CallbackConst<3>{}, FilterConst<3, Id>());
    f_reset_id_counters();
}

TEST(PhTreeTest, TestFilterAPI_FOR_EACH_WQ) {
    // Test edge case: only one entry in tree
    PhPointD<3> p{1, 2, 3};
    auto tree = TestTree<3, Id>();
    tree.emplace(p, Id{1});

    TestTree<3, Id>::QueryBox qb{{1, 2, 3}, {4, 5, 6}};
    CallbackCount<3> callback;
    FilterCount<3, Id> filter{};
    // lvalue
    tree.for_each(qb, callback, filter);
    ASSERT_EQ(static_id, 1);
    ASSERT_EQ(filter.last_known._i, 1);
    ASSERT_EQ(2, f_construct_ + f_default_construct_);
    ASSERT_EQ(0, f_copy_construct_ + f_move_construct_ + f_copy_assign_ + f_move_assign_);
    f_reset_id_counters();

    // rvalue
    tree.for_each({{1, 2, 3}, {4, 5, 6}}, CallbackCount<3>{}, FilterCount<3, Id>());
    ASSERT_EQ(static_id, 1);
    ASSERT_EQ(2, f_construct_ + f_default_construct_);
    ASSERT_GE(4, f_copy_construct_ + f_move_construct_ + f_copy_assign_ + f_move_assign_);
    f_reset_id_counters();

    // const Tree: just test that it compiles
    const TestTree<3, Id>& treeC = tree;
    // lvalue
    FilterConst<3, Id> filterC;
    treeC.for_each(qb, callback, filterC);
    // rvalue
    treeC.for_each({{1, 2, 3}, {4, 5, 6}}, CallbackConst<3>(), FilterConst<3, Id>());
    f_reset_id_counters();
}

TEST(PhTreeTest, TestFilterAPI_BEGIN) {
    // Test edge case: only one entry in tree
    PhPointD<3> p{1, 2, 3};
    auto tree = TestTree<3, Id>();
    tree.emplace(p, Id{1});

    FilterCount<3, Id> filter{};
    // lvalue
    ASSERT_EQ(tree.begin(filter)->_i, 1);
    ASSERT_EQ(filter.last_known._i, 1);
    ASSERT_EQ(1, f_construct_ + f_default_construct_);
    ASSERT_GE(0, f_copy_construct_ + f_move_construct_ + f_copy_assign_ + f_move_assign_);
    f_reset_id_counters();

    // rvalue
    ASSERT_EQ(tree.begin(FilterCount<3, Id>())->_i, 1);
    ASSERT_EQ(1, f_construct_ + f_default_construct_);
    ASSERT_GE(2, f_copy_construct_ + f_move_construct_ + f_copy_assign_ + f_move_assign_);
    f_reset_id_counters();

    // const Tree: just test that it compiles
    const TestTree<3, Id>& treeC = tree;
    // lvalue
    FilterConst<3, Id> filterC;
    ASSERT_EQ(treeC.begin(filterC)->_i, 1);
    // rvalue
    ASSERT_EQ(treeC.begin(FilterConst<3, Id>())->_i, 1);
    f_reset_id_counters();
}

TEST(PhTreeTest, TestFilterAPI_WQ) {
    // Test edge case: only one entry in tree
    PhPointD<3> p{1, 2, 3};
    auto tree = TestTree<3, Id>();
    tree.emplace(p, Id{1});

    TestTree<3, Id>::QueryBox qb{{1, 2, 3}, {4, 5, 6}};
    FilterCount<3, Id> filter{};
    // lvalue
    ASSERT_EQ(tree.begin_query(qb, filter)->_i, 1);
    ASSERT_EQ(filter.last_known._i, 1);
    ASSERT_EQ(1, f_construct_ + f_default_construct_);
    ASSERT_EQ(0, f_copy_construct_ + f_move_construct_ + f_copy_assign_ + f_move_assign_);
    f_reset_id_counters();

    // rvalue
    ASSERT_EQ(tree.begin_query({{1, 2, 3}, {4, 5, 6}}, FilterCount<3, Id>())->_i, 1);
    ASSERT_EQ(1, f_construct_ + f_default_construct_);
    ASSERT_GE(2, f_copy_construct_ + f_move_construct_ + f_copy_assign_ + f_move_assign_);
    f_reset_id_counters();

    // const Tree: just test that it compiles
    const TestTree<3, Id>& treeC = tree;
    // lvalue
    FilterConst<3, Id> filterC;
    ASSERT_EQ(treeC.begin_query(qb, filterC)->_i, 1);
    // rvalue
    ASSERT_EQ(treeC.begin_query(qb, FilterConst<3, Id>())->_i, 1);
    f_reset_id_counters();
}

TEST(PhTreeTest, TestFilterAPI_KNN) {
    // Test edge case: only one entry in tree
    PhPointD<3> p{1, 2, 3};
    auto tree = TestTree<3, Id>();
    tree.emplace(p, Id{1});

    FilterCount<3, Id> filter{};
    DistanceCount<3> dist_fn{};
    // lvalue
    ASSERT_EQ(tree.begin_knn_query(3, {2, 3, 4}, dist_fn, filter)->_i, 1);
    ASSERT_EQ(filter.last_known._i, 1);
    ASSERT_EQ(2, f_construct_ + f_default_construct_);
    ASSERT_EQ(0, f_copy_construct_ + f_move_construct_ + f_copy_assign_ + f_move_assign_);
    f_reset_id_counters();

    // rvalue
    ASSERT_EQ(tree.begin_knn_query(3, {2, 3, 4}, DistanceCount<3>{}, FilterCount<3, Id>())->_i, 1);
    ASSERT_EQ(2, f_construct_ + f_default_construct_);
    ASSERT_GE(2 * 3, f_copy_construct_ + f_move_construct_ + f_copy_assign_ + f_move_assign_);
    f_reset_id_counters();

    // rvalue #2
    auto a = tree.begin_knn_query<DistanceCount<3>, FilterCount<3, Id>>(3, {2, 3, 4})->_i;
    ASSERT_EQ(a, 1);
    ASSERT_EQ(2, f_construct_ + f_default_construct_);
    ASSERT_GE(2 * 3, f_copy_construct_ + f_move_construct_ + f_copy_assign_ + f_move_assign_);
    f_reset_id_counters();

    // const Tree: just test that it compiles
    const TestTree<3, Id>& treeC = tree;
    // lvalue
    FilterConst<3, Id> filterC;
    ASSERT_EQ(treeC.begin_knn_query(3, {2, 3, 4}, dist_fn, filterC)->_i, 1);
    // rvalue
    ASSERT_EQ(treeC.begin_knn_query(3, {2, 3, 4}, DistanceCount<3>{}, FilterConst<3, Id>())->_i, 1);
    f_reset_id_counters();
}

template <dimension_t DIM>
double distance(const TestPoint<DIM>& p1, const TestPoint<DIM>& p2) {
    double sum2 = 0;
    for (dimension_t i = 0; i < DIM; ++i) {
        double d2 = p1[i] - p2[i];
        sum2 += d2 * d2;
    }
    return sqrt(sum2);
};

template <dimension_t DIM>
void referenceSphereQuery(
    std::vector<TestPoint<DIM>>& points,
    TestPoint<DIM>& center,
    double radius,
    std::set<size_t>& result) {
    for (size_t i = 0; i < points.size(); i++) {
        auto& p = points[i];
        if (distance(center, p) <= radius) {
            result.insert(i);
        }
    }
}

template <dimension_t DIM>
void referenceAABBQuery(
    std::vector<TestPoint<DIM>>& points,
    TestPoint<DIM>& center,
    double radius,
    std::set<size_t>& result) {
    for (size_t i = 0; i < points.size(); i++) {
        auto& p = points[i];
        bool inside = true;
        for (dimension_t i = 0; i < DIM; ++i) {
            inside &= std::abs(p[i] - center[i]) <= radius;
        }

        if (inside) {
            result.insert(i);
        }
    }
}

template <dimension_t DIM>
PhBoxD<DIM> QueryBox(PhPointD<DIM>& center, double radius) {
    typename TestTree<DIM, size_t>::QueryBox query_box{
        {center[0] - radius, center[1] - radius, center[2] - radius},
        {center[0] + radius, center[1] + radius, center[2] + radius}};
    return query_box;
}

// We use 'int&' because gtest does not compile with assertions in non-void functions.
template <dimension_t DIM>
void testSphereQuery(TestPoint<DIM>& center, double radius, size_t N, int& result) {
    TestTree<DIM, size_t> tree;
    std::vector<TestPoint<DIM>> points;
    populate(tree, points, N);

    std::set<size_t> referenceResult;
    referenceSphereQuery(points, center, radius, referenceResult);

    result = 0;
    auto filter = FilterMultiMapSphere(center, radius, tree.converter());
    for (auto it = tree.begin(filter); it != tree.end(); it++) {
        auto& x = *it;
        ASSERT_GE(x, 0);
        ASSERT_EQ(referenceResult.count(x), 1);
        result++;
    }
    ASSERT_EQ(referenceResult.size(), result);
}

template <dimension_t DIM>
void testSphereQueryWithBox(TestPoint<DIM>& center, double radius, size_t N, int& result) {
    TestTree<DIM, size_t> tree;
    std::vector<TestPoint<DIM>> points;
    populate(tree, points, N);

    std::set<size_t> referenceResult;
    referenceSphereQuery(points, center, radius, referenceResult);

    result = 0;
    auto query_box = QueryBox(center, radius);
    auto filter = FilterMultiMapSphere(center, radius, tree.converter());
    for (auto it = tree.begin_query(query_box, filter); it != tree.end(); it++) {
        auto& x = *it;
        ASSERT_GE(x, 0);
        ASSERT_EQ(referenceResult.count(x), 1);
        result++;
    }
    ASSERT_EQ(referenceResult.size(), result);
}

template <dimension_t DIM>
void testSphereQueryForEach(TestPoint<DIM>& center, double radius, size_t N, int& result) {
    TestTree<DIM, size_t> tree;
    std::vector<TestPoint<DIM>> points;
    populate(tree, points, N);

    std::set<size_t> referenceResult;
    referenceSphereQuery(points, center, radius, referenceResult);

    result = 0;
    auto filter = FilterMultiMapSphere(center, radius, tree.converter());
    auto callback = [&result, &referenceResult](PhPointD<DIM>, const size_t& x) {
        ASSERT_GE(x, 0);
        ASSERT_EQ(referenceResult.count(x), 1);
        ++result;
    };
    tree.for_each(callback, filter);
    ASSERT_EQ(referenceResult.size(), result);
}
template <dimension_t DIM>
void testSphereQueryForEachQueryBox(TestPoint<DIM>& center, double radius, size_t N, int& result) {
    TestTree<DIM, size_t> tree;
    std::vector<TestPoint<DIM>> points;
    populate(tree, points, N);

    std::set<size_t> referenceResult;
    referenceSphereQuery(points, center, radius, referenceResult);

    result = 0;
    auto query_box = QueryBox(center, radius);
    auto filter = FilterMultiMapSphere(center, radius, tree.converter());
    auto callback = [&result, &referenceResult](PhPointD<DIM>, const size_t& x) {
        ASSERT_GE(x, 0);
        ASSERT_EQ(referenceResult.count(x), 1);
        ++result;
    };
    tree.for_each(query_box, callback, filter);
    ASSERT_EQ(referenceResult.size(), result);
}

template <dimension_t DIM>
void testAABBQuery(TestPoint<DIM>& center, double radius, size_t N, int& result) {
    TestTree<DIM, size_t> tree;
    std::vector<TestPoint<DIM>> points;
    populate(tree, points, N);

    std::set<size_t> referenceResult;
    referenceAABBQuery(points, center, radius, referenceResult);

    result = 0;
    auto query_box = QueryBox(center, radius);
    auto filter = FilterMultiMapAABB(query_box.min(), query_box.max(), tree.converter());
    for (auto it = tree.begin(filter); it != tree.end(); it++) {
        auto& x = *it;
        ASSERT_GE(x, 0);
        ASSERT_EQ(referenceResult.count(x), 1);
        result++;
    }
    ASSERT_EQ(referenceResult.size(), result);
}

template <dimension_t DIM, typename QUERY>
void Query0(QUERY query) {
    TestPoint<DIM> p{-10000, -10000, -10000};
    int n = 0;
    query(p, 0.1, 100, n);
    ASSERT_EQ(0, n);
}

template <dimension_t DIM, typename QUERY>
void QueryMany(QUERY query) {
    TestPoint<DIM> p{0, 0, 0};
    int n = 0;
    query(p, 1000, 1000, n);
    ASSERT_GT(n, 400);
    ASSERT_LT(n, 800);
}

template <dimension_t DIM, typename QUERY>
void QueryManyAABB(QUERY query) {
    TestPoint<DIM> p{0, 0, 0};
    int n = 0;
    query(p, 1000, 1000, n);
    ASSERT_EQ(n, 1000);
}

template <dimension_t DIM, typename QUERY>
void QueryAll(QUERY query) {
    TestPoint<DIM> p{0, 0, 0};
    int n = 0;
    query(p, 10000, 1000, n);
    ASSERT_EQ(1000, n);
}

TEST(PhTreeMMDFilterTest, TestSphereQuery) {
    Query0<3>(&testSphereQuery<3>);
    QueryMany<3>(&testSphereQuery<3>);
    QueryAll<3>(&testSphereQuery<3>);
}

TEST(PhTreeMMDFilterTest, TestSphereQueryWithQueryBox) {
    Query0<3>(&testSphereQueryWithBox<3>);
    QueryMany<3>(&testSphereQueryWithBox<3>);
    QueryAll<3>(&testSphereQueryWithBox<3>);
}

TEST(PhTreeMMDFilterTest, TestSphereQueryForEach) {
    Query0<3>(&testSphereQueryForEach<3>);
    QueryMany<3>(&testSphereQueryForEach<3>);
    QueryAll<3>(&testSphereQueryForEach<3>);
}

TEST(PhTreeMMDFilterTest, TestSphereQueryForEachWithQueryBox) {
    Query0<3>(&testSphereQueryForEachQueryBox<3>);
    QueryMany<3>(&testSphereQueryForEachQueryBox<3>);
    QueryAll<3>(&testSphereQueryForEachQueryBox<3>);
}

TEST(PhTreeMMDFilterTest, TestAABBQuery) {
    Query0<3>(&testAABBQuery<3>);
    QueryManyAABB<3>(&testAABBQuery<3>);
    QueryAll<3>(&testAABBQuery<3>);
}

}  // namespace phtree_multimap_d_test_filter
