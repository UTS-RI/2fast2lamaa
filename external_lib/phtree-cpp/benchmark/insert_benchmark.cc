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

using namespace improbable;
using namespace improbable::phtree;
using namespace improbable::phtree::phbenchmark;

namespace {

const int GLOBAL_MAX = 10000;

enum InsertionType {
    INSERT,
    EMPLACE,
    SQUARE_BR,
};

/*
 * Benchmark for adding entries to the index.
 */
template <dimension_t DIM, InsertionType TYPE>
class IndexBenchmark {
    using Index = PhTree<DIM, int>;

  public:
    explicit IndexBenchmark(benchmark::State& state);

    void Benchmark(benchmark::State& state);

  private:
    void SetupWorld(benchmark::State& state);

    void Insert(benchmark::State& state, Index& tree);

    const TestGenerator data_type_;
    const size_t num_entities_;
    std::vector<PhPoint<DIM>> points_;
};

template <dimension_t DIM, InsertionType TYPE>
IndexBenchmark<DIM, TYPE>::IndexBenchmark(benchmark::State& state)
: data_type_{static_cast<TestGenerator>(state.range(1))}
, num_entities_(state.range(0))
, points_(state.range(0)) {
    logging::SetupDefaultLogging();
    SetupWorld(state);
}

template <dimension_t DIM, InsertionType TYPE>
void IndexBenchmark<DIM, TYPE>::Benchmark(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto* tree = new Index();
        state.ResumeTiming();

        Insert(state, *tree);

        // we do this top avoid measuring deallocation
        state.PauseTiming();
        delete tree;
        state.ResumeTiming();
    }
}

template <dimension_t DIM, InsertionType TYPE>
void IndexBenchmark<DIM, TYPE>::SetupWorld(benchmark::State& state) {
    logging::info("Setting up world with {} entities and {} dimensions.", num_entities_, DIM);
    CreatePointData<DIM>(points_, data_type_, num_entities_, 0, GLOBAL_MAX);

    state.counters["total_put_count"] = benchmark::Counter(0);
    state.counters["put_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);

    logging::info("World setup complete.");
}

template <dimension_t DIM, InsertionType TYPE>
void IndexBenchmark<DIM, TYPE>::Insert(benchmark::State& state, Index& tree) {
    switch (TYPE) {
    case INSERT: {
        for (size_t i = 0; i < num_entities_; ++i) {
            tree.insert(points_[i], (int)i);
        }
        break;
    }
    case EMPLACE: {
        for (size_t i = 0; i < num_entities_; ++i) {
            tree.emplace(points_[i], (int)i);
        }
        break;
    }
    case SQUARE_BR: {
        for (size_t i = 0; i < num_entities_; ++i) {
            tree[points_[i]] = (int)i;
        }
        break;
    }
    }

    state.counters["total_put_count"] += num_entities_;
    state.counters["put_rate"] += num_entities_;
}

}  // namespace

template <typename... Arguments>
void PhTree3D_INS(benchmark::State& state, Arguments&&...) {
    IndexBenchmark<3, INSERT> benchmark{state};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void PhTree3D_EMP(benchmark::State& state, Arguments&&...) {
    IndexBenchmark<3, EMPLACE> benchmark{state};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void PhTree3D_SQB(benchmark::State& state, Arguments&&...) {
    IndexBenchmark<3, SQUARE_BR> benchmark{state};
    benchmark.Benchmark(state);
}

// index type, scenario name, data_generator, num_entities, function_to_call
BENCHMARK_CAPTURE(PhTree3D_INS, INSERT, 0)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 10 * 1000 * 1000}, {TestGenerator::CLUSTER, TestGenerator::CUBE}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D_EMP, EMPLACE, 0)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 10 * 1000 * 1000}, {TestGenerator::CLUSTER, TestGenerator::CUBE}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D_SQB, SQUARE_BR, 0)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 10 * 1000 * 1000}, {TestGenerator::CLUSTER, TestGenerator::CUBE}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
