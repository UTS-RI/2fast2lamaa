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
using payload_t = std::uint32_t;

/*
 * Benchmark for removing entries.
 */
template <dimension_t DIM>
class IndexBenchmark {
  public:
    IndexBenchmark(benchmark::State& state, TestGenerator data_type, int num_entities);

    void Benchmark(benchmark::State& state);

  private:
    void SetupWorld(benchmark::State& state);
    void Insert(benchmark::State& state, PhTreeD<DIM, payload_t>& tree);
    void Remove(benchmark::State& state, PhTreeD<DIM, payload_t>& tree);

    const TestGenerator data_type_;
    const size_t num_entities_;

    std::default_random_engine random_engine_;
    std::uniform_real_distribution<> cube_distribution_;
    std::vector<PhPointD<DIM>> points_;
};

template <dimension_t DIM>
IndexBenchmark<DIM>::IndexBenchmark(
    benchmark::State& state, TestGenerator data_type, int num_entities)
: data_type_{data_type}
, num_entities_(num_entities)
, random_engine_{1}
, cube_distribution_{0, GLOBAL_MAX}
, points_(num_entities) {
    logging::SetupDefaultLogging();
    SetupWorld(state);
}

template <dimension_t DIM>
void IndexBenchmark<DIM>::Benchmark(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto* tree = new PhTreeD<DIM, payload_t>();
        Insert(state, *tree);
        state.ResumeTiming();

        Remove(state, *tree);

        state.PauseTiming();
        // avoid measuring deallocation
        delete tree;
        state.ResumeTiming();
    }
}

template <dimension_t DIM>
void IndexBenchmark<DIM>::SetupWorld(benchmark::State& state) {
    logging::info("Setting up world with {} entities and {} dimensions.", num_entities_, DIM);
    CreatePointData<DIM>(points_, data_type_, num_entities_, 0, GLOBAL_MAX);

    state.counters["total_remove_count"] = benchmark::Counter(0);
    state.counters["remove_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);

    logging::info("World setup complete.");
}

template <dimension_t DIM>
void IndexBenchmark<DIM>::Insert(benchmark::State&, PhTreeD<DIM, payload_t>& tree) {
    for (payload_t i = 0; i < num_entities_; ++i) {
        tree.emplace(points_[i], i);
    }
}

template <dimension_t DIM>
void IndexBenchmark<DIM>::Remove(benchmark::State& state, PhTreeD<DIM, payload_t>& tree) {
    size_t n = 0;
    for (size_t i = 0; i < num_entities_; ++i) {
        n += tree.erase(points_[i]);
    }

    state.counters["total_remove_count"] += n;
    state.counters["remove_rate"] += n;
}

}  // namespace

template <typename... Arguments>
void PhTree3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

// index type, scenario name, data_generator, num_entities
// PhTree 3D CUBE
BENCHMARK_CAPTURE(PhTree3D, REM_CU_1K, TestGenerator::CUBE, 1000)->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, REM_CU_10K, TestGenerator::CUBE, 10000)->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, REM_CU_100K, TestGenerator::CUBE, 100000)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, REM_CU_1M, TestGenerator::CUBE, 1000000)->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, REM_CU_10M, TestGenerator::CUBE, 10000000)
    ->Unit(benchmark::kMillisecond);

// index type, scenario name, data_generator, num_entities
// PhTree 3D CLUSTER
BENCHMARK_CAPTURE(PhTree3D, REM_CL_1K, TestGenerator::CLUSTER, 1000)->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, REM_CL_10K, TestGenerator::CLUSTER, 10000)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, REM_CL_100K, TestGenerator::CLUSTER, 100000)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, REM_CL_1M, TestGenerator::CLUSTER, 1000000)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, REM_CL_10M, TestGenerator::CLUSTER, 10000000)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
