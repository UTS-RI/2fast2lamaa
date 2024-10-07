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

const int GLOBAL_MAX = 10000;

enum QueryType {
    FIND,
    COUNT,
};

/*
 * Benchmark for looking up entries by their key.
 */
template <dimension_t DIM, QueryType QUERY_TYPE>
class IndexBenchmark {
  public:
    IndexBenchmark(benchmark::State& state, double dummy);

    void Benchmark(benchmark::State& state);

  private:
    void SetupWorld(benchmark::State& state);
    int QueryWorldCount();
    int QueryWorldFind();

    const TestGenerator data_type_;
    const size_t num_entities_;
    const QueryType query_type_;

    PhTree<DIM, int> tree_;
    std::default_random_engine random_engine_;
    std::uniform_int_distribution<> cube_distribution_;
    std::vector<PhPoint<DIM>> points_;
};

template <dimension_t DIM, QueryType QUERY_TYPE>
IndexBenchmark<DIM, QUERY_TYPE>::IndexBenchmark(benchmark::State& state, double)
: data_type_{static_cast<TestGenerator>(state.range(1))}
, num_entities_(state.range(0))
, query_type_(QUERY_TYPE)
, random_engine_{1}
, cube_distribution_{0, GLOBAL_MAX}
, points_(state.range(0)) {
    logging::SetupDefaultLogging();
    SetupWorld(state);
}

template <dimension_t DIM, QueryType QUERY_TYPE>
void IndexBenchmark<DIM, QUERY_TYPE>::Benchmark(benchmark::State& state) {
    int num_inner = 0;
    int num_found = 0;
    switch (query_type_) {
    case COUNT: {
        for (auto _ : state) {
            num_found += QueryWorldCount();
            ++num_inner;
        }
        break;
    }
    case FIND: {
        for (auto _ : state) {
            num_found += QueryWorldFind();
            ++num_inner;
        }
        break;
    }
    }
    // Moved outside of the loop because EXPENSIVE
    state.counters["total_result_count"] += num_found;
    state.counters["query_rate"] += num_inner;
    state.counters["result_rate"] += num_found;
    state.counters["avg_result_count"] += num_found;
}

template <dimension_t DIM, QueryType QUERY_TYPE>
void IndexBenchmark<DIM, QUERY_TYPE>::SetupWorld(benchmark::State& state) {
    logging::info("Setting up world with {} entities and {} dimensions.", num_entities_, DIM);
    CreatePointData<DIM>(points_, data_type_, num_entities_, 0, GLOBAL_MAX);
    for (size_t i = 0; i < num_entities_; ++i) {
        tree_.emplace(points_[i], (int)i);
    }

    state.counters["total_result_count"] = benchmark::Counter(0);
    state.counters["query_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);
    state.counters["result_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);
    state.counters["avg_result_count"] = benchmark::Counter(0, benchmark::Counter::kAvgIterations);
    logging::info("World setup complete.");
}

template <dimension_t DIM, QueryType QUERY_TYPE>
int IndexBenchmark<DIM, QUERY_TYPE>::QueryWorldCount() {
    static int pos = 0;
    pos = (pos + 1) % num_entities_;
    bool found;
    if (pos % 2 == 0) {
        found = tree_.count(points_.at(pos));
        assert(found);
    } else {
        int x = pos % GLOBAL_MAX;
        PhPoint<DIM> p = PhPoint<DIM>({x, x, x});
        found = tree_.count(p);
    }
    return found;
}

template <dimension_t DIM, QueryType QUERY_TYPE>
int IndexBenchmark<DIM, QUERY_TYPE>::QueryWorldFind() {
    static int pos = 0;
    pos = (pos + 1) % num_entities_;
    bool found;
    if (pos % 2 == 0) {
        // This should always be a match
        found = tree_.find(points_.at(pos)) != tree_.end();
        assert(found);
    } else {
        // This should rarely be a match
        int x = pos % GLOBAL_MAX;
        PhPoint<DIM> p = PhPoint<DIM>({x, x, x});
        found = tree_.find(p) != tree_.end();
    }
    return found;
}

}  // namespace

template <typename... Arguments>
void PhTree3DCount(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, QueryType::COUNT> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void PhTree3DFind(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, QueryType::FIND> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

// index type, scenario name, data_generator, num_entities, function_to_call
BENCHMARK_CAPTURE(PhTree3DCount, COUNT, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3DFind, FIND, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
