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
#include "benchmark_util.h"
#include "logging.h"
#include "phtree/phtree.h"
#include "phtree/phtree_multimap.h"
#include <benchmark/benchmark.h>
#include <random>

using namespace improbable;
using namespace improbable::phtree;
using namespace improbable::phtree::phbenchmark;

/*
 * Benchmark for k-nearest-neighbour queries in multi-map implementations.
 */
namespace {

const double GLOBAL_MAX = 10000;
const dimension_t DIM = 3;

enum Scenario {
    TREE_SET,
    PHTREE_MM,
    PHTREE_MM_STD,
    PHTREE2,
    TS_KD,
    TS_QT,
};

using payload_t = int64_t;
using payload2_t = uint32_t;

using TestPoint = PhPointD<DIM>;
using QueryBox = PhBoxD<DIM>;
using BucketType = std::set<payload_t>;

template <dimension_t DIM>
using CONVERTER = ConverterIEEE<DIM>;

template <Scenario SCENARIO, dimension_t DIM>
using TestMap = typename std::conditional_t<
    SCENARIO == TREE_SET,
    PhTreeD<DIM, BucketType, CONVERTER<DIM>>,
    typename std::conditional_t<
        SCENARIO == PHTREE_MM,
        PhTreeMultiMap<DIM, payload_t, CONVERTER<DIM>, b_plus_tree_hash_set<payload_t>>,
//        typename std::conditional_t<
//            SCENARIO == PHTREE2,
//            PhTreeMultiMap2D<DIM, payload_t>,
            typename std::conditional_t<
                SCENARIO == PHTREE_MM_STD,
                PhTreeMultiMap<DIM, payload_t, CONVERTER<DIM>, BucketType>,
                void>>>;

template <dimension_t DIM, Scenario SCENARIO>
class IndexBenchmark {
  public:
    IndexBenchmark(benchmark::State& state, int knn_result_size_);

    void Benchmark(benchmark::State& state);

  private:
    void SetupWorld(benchmark::State& state);
    void QueryWorld(benchmark::State& state, TestPoint& center);
    void CreateQuery(TestPoint& center);

    const TestGenerator data_type_;
    const size_t num_entities_;
    const size_t knn_result_size_;

    TestMap<SCENARIO, DIM> tree_;
    std::default_random_engine random_engine_;
    std::uniform_real_distribution<> cube_distribution_;
    std::vector<TestPoint> points_;
};

template <dimension_t DIM, Scenario SCENARIO>
IndexBenchmark<DIM, SCENARIO>::IndexBenchmark(benchmark::State& state, int knn_result_size)
: data_type_{static_cast<TestGenerator>(state.range(1))}
, num_entities_(state.range(0))
, knn_result_size_(knn_result_size)
, random_engine_{1}
, cube_distribution_{0, GLOBAL_MAX}
, points_(state.range(0)) {
    logging::SetupDefaultLogging();
    SetupWorld(state);
}

template <dimension_t DIM, Scenario SCENARIO>
void IndexBenchmark<DIM, SCENARIO>::Benchmark(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        TestPoint center;
        CreateQuery(center);
        state.ResumeTiming();

        QueryWorld(state, center);
    }
}

template <
    dimension_t DIM,
    Scenario SCENARIO,
    std::enable_if_t<(SCENARIO == Scenario::TREE_SET), int> = 0>
void InsertEntries(TestMap<Scenario::TREE_SET, DIM>& tree, const std::vector<TestPoint>& points) {
    for (size_t i = 0; i < points.size(); ++i) {
        BucketType& bucket = tree.emplace(points[i]).first;
        bucket.emplace((payload_t)i);
    }
}

template <dimension_t DIM, Scenario SCN, std::enable_if_t<(SCN != Scenario::TREE_SET), int> = 0>
void InsertEntries(TestMap<SCN, DIM>& tree, const std::vector<TestPoint>& points) {
    for (size_t i = 0; i < points.size(); ++i) {
        tree.emplace(points[i], (payload_t)i);
    }
}

template <dimension_t DIM, Scenario SCN>
size_t QueryAll(TestMap<SCN, DIM>& tree, const TestPoint& center, const size_t k) {
    size_t n = 0;
    for (auto q = tree.begin_knn_query(k, center, DistanceEuclidean<DIM>()); q != tree.end(); ++q) {
        ++n;
    }
    return n;
}

struct CounterTreeWithMap {
    void operator()(const TestPoint&, const BucketType& value) {
        for (auto& x : value) {
            (void)x;
            n_ += 1;
        }
    }
    size_t n_;
};

struct CounterMultiMap {
    void operator()(const TestPoint&, const payload_t&) {
        n_ += 1;
    }
    size_t n_;
};

template <dimension_t DIM, Scenario SCENARIO>
void IndexBenchmark<DIM, SCENARIO>::SetupWorld(benchmark::State& state) {
    logging::info("Setting up world with {} entities and {} dimensions.", num_entities_, DIM);
    CreatePointData<DIM>(points_, data_type_, num_entities_, 0, GLOBAL_MAX);
    InsertEntries<DIM, SCENARIO>(tree_, points_);

    state.counters["query_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);
    state.counters["result_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);
    state.counters["avg_result_count"] = benchmark::Counter(0, benchmark::Counter::kAvgIterations);
    logging::info("World setup complete.");
}

template <dimension_t DIM, Scenario SCENARIO>
void IndexBenchmark<DIM, SCENARIO>::QueryWorld(benchmark::State& state, TestPoint& center) {
    size_t n = QueryAll<DIM, SCENARIO>(tree_, center, knn_result_size_);

    state.counters["query_rate"] += 1;
    state.counters["result_rate"] += n;
    state.counters["avg_result_count"] += n;
}

template <dimension_t DIM, Scenario SCENARIO>
void IndexBenchmark<DIM, SCENARIO>::CreateQuery(TestPoint& center) {
    for (dimension_t d = 0; d < DIM; ++d) {
        center[d] = cube_distribution_(random_engine_) * GLOBAL_MAX;
    }
}

}  // namespace

// template <typename... Arguments>
// void TinspinKDTree(benchmark::State& state, Arguments&&... arguments) {
//     IndexBenchmark<DIM, Scenario::TS_KD> benchmark{state, arguments...};
//     benchmark.Benchmark(state);
// }
//
// template <typename... Arguments>
// void TinspinQuadtree(benchmark::State& state, Arguments&&... arguments) {
//     IndexBenchmark<DIM, Scenario::TS_QT> benchmark{state, arguments...};
//     benchmark.Benchmark(state);
// }

template <typename... Arguments>
void PhTree3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<DIM, Scenario::TREE_SET> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void PhTreeMM(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<DIM, Scenario::PHTREE_MM> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

//template <typename... Arguments>
//void PhTreeMM2(benchmark::State& state, Arguments&&... arguments) {
//    IndexBenchmark<DIM, Scenario::PHTREE2> benchmark{state, arguments...};
//    benchmark.Benchmark(state);
//}

template <typename... Arguments>
void PhTreeMMStdSet(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<DIM, Scenario::PHTREE_MM_STD> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

// index type, scenario name, data_type, num_entities, query_result_size

// PhTree multi-map 1.0
BENCHMARK_CAPTURE(PhTreeMM, KNN_1, 1)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTreeMM, KNN_10, 10)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

//// Multimap 2.0
//BENCHMARK_CAPTURE(PhTreeMM2, KNN_1, 1)
//    ->RangeMultiplier(10)
//    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
//    ->Unit(benchmark::kMillisecond);
//
//BENCHMARK_CAPTURE(PhTreeMM2, KNN_10, 10)
//    ->RangeMultiplier(10)
//    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
//    ->Unit(benchmark::kMillisecond);

//// KD-tree
// BENCHMARK_CAPTURE(TinspinKDTree, KNN_1, 1)
//     ->RangeMultiplier(10)
//     ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
//     ->Unit(benchmark::kMillisecond);
//
// BENCHMARK_CAPTURE(TinspinKDTree, KNN_10, 10)
//     ->RangeMultiplier(10)
//     ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
//     ->Unit(benchmark::kMillisecond);
//
//// Quadtree
// BENCHMARK_CAPTURE(TinspinQuadtree, KNN_1, 1)
//     ->RangeMultiplier(10)
//     ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
//     ->Unit(benchmark::kMillisecond);
//
// BENCHMARK_CAPTURE(TinspinQuadtree, KNN_10, 10)
//     ->RangeMultiplier(10)
//     ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
//     ->Unit(benchmark::kMillisecond);

//  PhTree 3D with set
BENCHMARK_CAPTURE(PhTree3D, KNN_1, 1)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, KNN_10, 10)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
