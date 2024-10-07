/*
 * Copyright 2022-2023 Tilmann Zäschke
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
#include "phtree/common/b_plus_tree_hash_map.h"
#include "phtree/common/b_plus_tree_map.h"
#include "phtree/common/b_plus_tree_multimap.h"
#include <benchmark/benchmark.h>

using namespace improbable::phtree::phbenchmark;
using namespace phtree::bptree;

namespace {

const int GLOBAL_MAX = 10000;

enum Scenario {
    MAP,
    MULTIMAP,
    HASH_MAP,
    STD_MAP,
    STD_MULTIMAP,
};

using payload_t = int;
using key_t = uint32_t;
template <size_t DIM>
using TestPoint = std::array<double, DIM>;

template <Scenario SCENARIO, size_t MAX_SIZE>
using TestIndex = typename std::conditional_t<
    SCENARIO == MAP,
    b_plus_tree_map<std::uint64_t, payload_t, MAX_SIZE>,
    typename std::conditional_t<
        SCENARIO == MULTIMAP,
        b_plus_tree_multimap<key_t, payload_t>,
        typename std::conditional_t<
            SCENARIO == HASH_MAP,
            b_plus_tree_hash_map<key_t, payload_t>,
            typename std::conditional_t<
                SCENARIO == STD_MAP,
                std::map<key_t, payload_t>,
                typename std::conditional_t<
                    SCENARIO == STD_MULTIMAP,
                    std::multimap<key_t, payload_t>,
                    void>>>>>;

/*
 * Benchmark for removing entries.
 */
template <size_t DIM, Scenario TYPE>
class IndexBenchmark {
    using Index = TestIndex<TYPE, (1 << DIM)>;

  public:
    explicit IndexBenchmark(benchmark::State& state, double fraction_of_duplicates);
    void Benchmark(benchmark::State& state);

  private:
    void SetupWorld(benchmark::State& state);
    void Insert(benchmark::State& state, Index& tree);
    void Remove(benchmark::State& state, Index& tree);

    const TestGenerator data_type_;
    const size_t num_entities_;
    const double fraction_of_duplicates_;

    std::default_random_engine random_engine_;
    std::uniform_int_distribution<> cube_distribution_;
    std::vector<TestPoint<1>> points_;
};

template <size_t DIM, Scenario TYPE>
IndexBenchmark<DIM, TYPE>::IndexBenchmark(benchmark::State& state, double fraction_of_duplicates)
: data_type_{static_cast<TestGenerator>(state.range(1))}
, num_entities_(state.range(0))
, fraction_of_duplicates_(fraction_of_duplicates)
, random_engine_{1}
, cube_distribution_{0, GLOBAL_MAX}
, points_(state.range(0)) {
    logging::SetupDefaultLogging();
    SetupWorld(state);
}

template <size_t DIM, Scenario TYPE>
void IndexBenchmark<DIM, TYPE>::Benchmark(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        auto* tree = new Index();
        Insert(state, *tree);
        state.ResumeTiming();

        Remove(state, *tree);

        state.PauseTiming();
        // avoid measuring deallocation
        delete tree;
        state.ResumeTiming();
    }
}

template <size_t DIM, Scenario TYPE>
void IndexBenchmark<DIM, TYPE>::SetupWorld(benchmark::State& state) {
    logging::info("Creating {} entities with DIM={}.", num_entities_, 1);
    CreatePointData<1>(points_, data_type_, num_entities_, 0, GLOBAL_MAX, fraction_of_duplicates_);

    state.counters["total_remove_count"] = benchmark::Counter(0);
    state.counters["remove_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);
    logging::info("World setup complete.");
}

template <size_t DIM, Scenario TYPE>
void IndexBenchmark<DIM, TYPE>::Insert(benchmark::State&, Index& tree) {
    for (size_t i = 0; i < num_entities_; ++i) {
        tree.emplace(points_[i][0], (payload_t)i);
    }
}

template <size_t DIM, Scenario TYPE>
void IndexBenchmark<DIM, TYPE>::Remove(benchmark::State& state, Index& tree) {
    size_t n = 0;
    for (size_t i = 0; i < num_entities_; ++i) {
        tree.erase(points_[i][0]);
        ++n;
    }

    state.counters["total_remove_count"] += n;
    state.counters["remove_rate"] += n;
}

}  // namespace

template <typename... Arguments>
void BPT_MAP_REM(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, MAP> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void BPT_MM_REM(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, MULTIMAP> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void BPT_HM_REM(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, HASH_MAP> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void STD_MULTIMAP_REM(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, STD_MULTIMAP> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void STD_MAP_REM(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, STD_MAP> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

// index type, scenario name, data_generator, num_entities, function_to_call
BENCHMARK_CAPTURE(BPT_MAP_REM, MAP, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{100, 100 * 1000}, {TestGenerator::CLUSTER, TestGenerator::CUBE}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BPT_MM_REM, MULTIMAP, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{100, 100 * 1000}, {TestGenerator::CLUSTER, TestGenerator::CUBE}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BPT_HM_REM, HASH_MAP, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{100, 100 * 1000}, {TestGenerator::CLUSTER, TestGenerator::CUBE}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(STD_MULTIMAP_REM, STD_MULTIMAP, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{100, 100 * 1000}, {TestGenerator::CLUSTER, TestGenerator::CUBE}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(STD_MAP_REM, STD_MAP, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{100, 100 * 1000}, {TestGenerator::CLUSTER, TestGenerator::CUBE}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
