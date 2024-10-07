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
#include <benchmark/benchmark.h>
#include <random>

using namespace improbable;
using namespace improbable::phtree;
using namespace improbable::phtree::phbenchmark;

namespace {

const double GLOBAL_MAX = 10000;
const double BOX_LEN = GLOBAL_MAX / 100.;

enum QueryType { MIN_MAX_ITER, MIN_MAX_FOR_EACH };

template <dimension_t DIM>
using BoxType = PhBoxD<DIM>;

template <dimension_t DIM>
using PointType = PhPointD<DIM>;

template <dimension_t DIM>
using TreeType = PhTreeBoxD<DIM, int>;

/*
 * Benchmark for window queries.
 */
template <dimension_t DIM, QueryType QUERY_TYPE>
class IndexBenchmark {
  public:
    IndexBenchmark(benchmark::State& state, double avg_query_result_size_ = 100);

    void Benchmark(benchmark::State& state);

  private:
    void SetupWorld(benchmark::State& state);
    void QueryWorld(benchmark::State& state, BoxType<DIM>& query_box);
    void CreateQuery(BoxType<DIM>& query_box);

    const TestGenerator data_type_;
    const size_t num_entities_;
    const double avg_query_result_size_;

    constexpr int query_edge_length() {
        return (
            int)(GLOBAL_MAX * pow(avg_query_result_size_ / (double)num_entities_, 1. / (double)DIM));
    };

    TreeType<DIM> tree_;
    std::default_random_engine random_engine_;
    std::uniform_real_distribution<> cube_distribution_;
    std::vector<BoxType<DIM>> boxes_;
};

template <dimension_t DIM, QueryType QUERY_TYPE>
IndexBenchmark<DIM, QUERY_TYPE>::IndexBenchmark(
    benchmark::State& state, double avg_query_result_size)
: data_type_{static_cast<TestGenerator>(state.range(1))}
, num_entities_(state.range(0))
, avg_query_result_size_(avg_query_result_size)
, tree_{}
, random_engine_{1}
, cube_distribution_{0, GLOBAL_MAX}
, boxes_(state.range(0)) {
    logging::SetupDefaultLogging();
    SetupWorld(state);
}

template <dimension_t DIM, QueryType QUERY_TYPE>
void IndexBenchmark<DIM, QUERY_TYPE>::Benchmark(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        BoxType<DIM> query_box;
        CreateQuery(query_box);
        state.ResumeTiming();

        QueryWorld(state, query_box);
    }
}

template <dimension_t DIM, QueryType QUERY_TYPE>
void IndexBenchmark<DIM, QUERY_TYPE>::SetupWorld(benchmark::State& state) {
    logging::info("Setting up world with {} entities and {} dimensions.", num_entities_, DIM);
    CreateBoxData<DIM>(boxes_, data_type_, num_entities_, 0, GLOBAL_MAX, BOX_LEN);
    for (size_t i = 0; i < num_entities_; ++i) {
        tree_.emplace(boxes_[i], (int)i);
    }

    state.counters["query_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);
    state.counters["result_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);
    state.counters["avg_result_count"] = benchmark::Counter(0, benchmark::Counter::kAvgIterations);
    logging::info("World setup complete.");
}

template <dimension_t DIM, typename T>
struct Counter {
    void operator()(BoxType<DIM>, T&) {
        ++n_;
    }

    size_t n_ = 0;
};

template <dimension_t DIM>
size_t Count_MMI(TreeType<DIM>& tree, BoxType<DIM>& query_box) {
    size_t n = 0;
    for (auto q = tree.begin_query(query_box); q != tree.end(); ++q) {
        ++n;
    }
    return n;
}

template <dimension_t DIM>
size_t Count_MMFE(TreeType<DIM>& tree, BoxType<DIM>& query_box) {
    Counter<DIM, int> callback;
    tree.for_each(query_box, callback);
    return callback.n_;
}

template <dimension_t DIM, QueryType QUERY_TYPE>
void IndexBenchmark<DIM, QUERY_TYPE>::QueryWorld(benchmark::State& state, BoxType<DIM>& query_box) {
    size_t n = 0;
    switch (QUERY_TYPE) {
    case MIN_MAX_ITER:
        n = Count_MMI(tree_, query_box);
        break;
    case MIN_MAX_FOR_EACH:
        n = Count_MMFE(tree_, query_box);
        break;
    }

    state.counters["query_rate"] += 1;
    state.counters["result_rate"] += n;
    state.counters["avg_result_count"] += n;
}

template <dimension_t DIM, QueryType QUERY_TYPE>
void IndexBenchmark<DIM, QUERY_TYPE>::CreateQuery(BoxType<DIM>& query_box) {
    int length = query_edge_length();
    // scale to ensure query lies within boundary
    double shift = (GLOBAL_MAX - (double)length) / GLOBAL_MAX;
    for (dimension_t d = 0; d < DIM; ++d) {
        auto s = shift * cube_distribution_(random_engine_);
        s = s * shift;
        query_box.min()[d] = s;
        query_box.max()[d] = s + length;
    }
}

}  // namespace

template <typename... Arguments>
void PhTree3D_MMI(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, MIN_MAX_ITER> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void PhTree3D_MMFE(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, MIN_MAX_FOR_EACH> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

// index type, scenario name, data_type, num_entities, query_result_size
// PhTree
BENCHMARK_CAPTURE(PhTree3D_MMFE, WQ_100, 100.0)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

// PhTree
BENCHMARK_CAPTURE(PhTree3D_MMI, WQ_100, 100.0)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
