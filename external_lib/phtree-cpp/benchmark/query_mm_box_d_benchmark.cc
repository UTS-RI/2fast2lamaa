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
 * Benchmark for querying entries in multi-map implementations.
 */
namespace {

const double GLOBAL_MAX = 10000;
const double BOX_LEN = 100;

enum Scenario { TREE_WITH_MAP, MULTI_MAP };

using TestPoint = PhPointD<3>;
using QueryBox = PhBoxD<3>;
using payload_t = PhBoxD<3>;
using BucketType = std::unordered_set<payload_t>;

struct Query {
    QueryBox box_{};
};

template <Scenario SCENARIO, dimension_t DIM>
using CONVERTER = ConverterBoxIEEE<DIM>;

template <Scenario SCENARIO, dimension_t DIM>
using TestMap = typename std::conditional_t<
    SCENARIO == TREE_WITH_MAP,
    PhTreeBoxD<DIM, BucketType, CONVERTER<SCENARIO, DIM>>,
    PhTreeMultiMapBoxD<DIM, payload_t, CONVERTER<SCENARIO, DIM>>>;

template <dimension_t DIM, Scenario SCENARIO>
class IndexBenchmark {
  public:
    IndexBenchmark(benchmark::State& state, double avg_query_result_size_);

    void Benchmark(benchmark::State& state);

  private:
    void SetupWorld(benchmark::State& state);
    void QueryWorld(benchmark::State& state, const Query& query);
    void CreateQuery(Query& query);

    const TestGenerator data_type_;
    const size_t num_entities_;
    const double avg_query_result_size_;

    constexpr double query_edge_length() {
        return GLOBAL_MAX * pow(avg_query_result_size_ / (double)num_entities_, 1. / (double)DIM);
    };

    TestMap<SCENARIO, DIM> tree_;
    std::default_random_engine random_engine_;
    std::uniform_real_distribution<> cube_distribution_;
    std::vector<PhBoxD<DIM>> boxes_;
};

template <dimension_t DIM, Scenario SCENARIO>
IndexBenchmark<DIM, SCENARIO>::IndexBenchmark(benchmark::State& state, double avg_query_result_size)
: data_type_{static_cast<TestGenerator>(state.range(1))}
, num_entities_(state.range(0))
, avg_query_result_size_(avg_query_result_size)
, tree_{}
, random_engine_{1}
, cube_distribution_{0, GLOBAL_MAX}
, boxes_(num_entities_) {
    logging::SetupDefaultLogging();
    SetupWorld(state);
}

template <dimension_t DIM, Scenario SCENARIO>
void IndexBenchmark<DIM, SCENARIO>::Benchmark(benchmark::State& state) {
    Query query{};
    for (auto _ : state) {
        state.PauseTiming();
        CreateQuery(query);
        state.ResumeTiming();

        QueryWorld(state, query);
    }
}

template <dimension_t DIM>
void InsertEntry(
    TestMap<Scenario::TREE_WITH_MAP, DIM>& tree, const PhBoxD<DIM>& point, const payload_t& data) {
    BucketType& bucket = tree.emplace(point).first;
    bucket.emplace(data);
}

template <dimension_t DIM>
void InsertEntry(
    TestMap<Scenario::MULTI_MAP, DIM>& tree, const PhBoxD<DIM>& point, const payload_t& data) {
    tree.emplace(point, data);
}

bool CheckPosition(const payload_t& entity, const QueryBox& query) {
    const auto& box = entity;
    bool result = true;
    for (int d = 0; d < 3; ++d) {
        result = query.min()[d] <= box.max()[0] && query.max()[d] >= box.min()[d];
    }
    return result;
}

struct CounterTreeWithMap {
    void operator()(const PhBoxD<3>&, const BucketType& value) {
        for (auto& x : value) {
            n_ += CheckPosition(x, box_);
        }
    }
    const QueryBox& box_;
    size_t n_;
};

struct CounterMultiMap {
    void operator()(const PhBoxD<3>&, const payload_t& value) {
        n_ += CheckPosition(value, box_);
    }
    const QueryBox& box_;
    size_t n_;
};

template <dimension_t DIM, Scenario SCENARIO>
typename std::enable_if<SCENARIO == Scenario::TREE_WITH_MAP, size_t>::type CountEntries(
    TestMap<Scenario::TREE_WITH_MAP, DIM>& tree, const Query& query) {
    CounterTreeWithMap counter{query.box_, 0};
    tree.for_each(query.box_, counter);
    return counter.n_;
}

template <dimension_t DIM, Scenario SCENARIO>
size_t CountEntries(TestMap<Scenario::MULTI_MAP, DIM>& tree, const Query& query) {
    CounterMultiMap counter{query.box_, 0};
    tree.for_each(query.box_, counter);
    return counter.n_;
}

template <dimension_t DIM, Scenario SCENARIO>
void IndexBenchmark<DIM, SCENARIO>::SetupWorld(benchmark::State& state) {
    logging::info("Setting up world with {} entities and {} dimensions.", num_entities_, DIM);
    // create data with about 10% duplicate coordinates
    CreateBoxData<DIM>(boxes_, data_type_, num_entities_, 0, GLOBAL_MAX, BOX_LEN, 0.1);
    for (size_t i = 0; i < num_entities_; ++i) {
        InsertEntry(tree_, boxes_[i], boxes_[i]);
    }

    state.counters["query_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);
    state.counters["result_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);
    state.counters["avg_result_count"] = benchmark::Counter(0, benchmark::Counter::kAvgIterations);
    logging::info("World setup complete.");
}

template <dimension_t DIM, Scenario SCENARIO>
void IndexBenchmark<DIM, SCENARIO>::QueryWorld(benchmark::State& state, const Query& query) {
    size_t n = CountEntries<DIM, SCENARIO>(tree_, query);

    state.counters["query_rate"] += 1;
    state.counters["result_rate"] += n;
    state.counters["avg_result_count"] += n;
}

template <dimension_t DIM, Scenario SCENARIO>
void IndexBenchmark<DIM, SCENARIO>::CreateQuery(Query& query) {
    double length = query_edge_length();
    // shift to ensure query lies within boundary
    double shift = (GLOBAL_MAX - (double)length) / GLOBAL_MAX;
    for (dimension_t d = 0; d < DIM; ++d) {
        auto x = shift * cube_distribution_(random_engine_);
        query.box_.min()[d] = x;
        query.box_.max()[d] = x + length;
    }
}

}  // namespace

template <typename... Arguments>
void PhTree3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, Scenario::TREE_WITH_MAP> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void PhTreeMultiMapM3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, Scenario::MULTI_MAP> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

// index type, scenario name, data_type, num_entities, avg_query_result_size
// PhTree
BENCHMARK_CAPTURE(PhTree3D, WQ_100, 100.0)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

// PhTreeMultiMap
BENCHMARK_CAPTURE(PhTreeMultiMapM3D, WQ_100, 100.0)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
