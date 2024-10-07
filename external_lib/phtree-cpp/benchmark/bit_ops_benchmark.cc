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
#include <benchmark/benchmark.h>

using namespace improbable::phtree;
using namespace improbable::phtree::detail;
using namespace improbable::phtree::phbenchmark;

namespace {

const int GLOBAL_MAX = 10000;

enum Scenario {
    POS_IN_ARRAY,
    IS_IN_RANGE,
    CALC_LIMITS,
};

/*
 * Benchmark for adding entries to the index.
 */
template <dimension_t DIM, Scenario TYPE>
class IndexBenchmark {
    using Point = PhPoint<DIM, std::uint64_t>;

  public:
    explicit IndexBenchmark(benchmark::State& state, double);
    void Benchmark(benchmark::State& state);

  private:
    void SetupWorld(benchmark::State& state);
    void Insert(benchmark::State& state);

    const TestGenerator data_type_;
    const size_t num_entities_;
    std::vector<Point> points_;
};

template <dimension_t DIM, Scenario TYPE>
IndexBenchmark<DIM, TYPE>::IndexBenchmark(benchmark::State& state, double)
: data_type_{static_cast<TestGenerator>(state.range(1))}
, num_entities_(state.range(0))
, points_(state.range(0)) {
    logging::SetupDefaultLogging();
    SetupWorld(state);
}

template <dimension_t DIM, Scenario TYPE>
void IndexBenchmark<DIM, TYPE>::Benchmark(benchmark::State& state) {
    for (auto _ : state) {
        Insert(state);
    }
}

template <dimension_t DIM, Scenario TYPE>
void IndexBenchmark<DIM, TYPE>::SetupWorld(benchmark::State& state) {
    logging::info("Creating {} entities with DIM={}.", num_entities_, 1);
    CreatePointData<1>(points_, data_type_, num_entities_, 0, GLOBAL_MAX);

    state.counters["total_op_count"] = benchmark::Counter(0);
    state.counters["op_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);
    logging::info("World setup complete.");
}

template <dimension_t DIM, Scenario TYPE>
void IndexBenchmark<DIM, TYPE>::Insert(benchmark::State& state) {
    switch (TYPE) {
    case POS_IN_ARRAY: {
        for (size_t i = 0; i < num_entities_; ++i) {
            bit_width_t postlen = i % 63;
            auto pos = CalcPosInArray(points_[i], postlen);
            benchmark::DoNotOptimize(pos);
        }
        break;
    }
    case IS_IN_RANGE: {
        for (size_t i = 0; i < num_entities_; ++i) {
            auto i2 = (i + 1) % num_entities_;
            auto i3 = (i + 2) % num_entities_;
            auto result = IsInRange(points_[i], points_[i2], points_[i3]);
            benchmark::DoNotOptimize(result);
        }
        break;
    }
    case CALC_LIMITS: {
        for (size_t i = 0; i < num_entities_; ++i) {
            auto i2 = (i + 1) % num_entities_;
            auto i3 = (i + 2) % num_entities_;
            hc_pos_dim_t<DIM> m1, m2;
            detail::CalcLimits(i % 63, points_[i], points_[i2], points_[i3], m1, m2);
            benchmark::DoNotOptimize(m1);
            benchmark::DoNotOptimize(m2);
        }
        break;
    }
    default:
        logging::error("Invalid scenario.");
        exit(1);
    }

    state.counters["total_op_count"] += num_entities_;
    state.counters["op_rate"] += num_entities_;
}

}  // namespace

template <typename... Arguments>
void POS_IN_ARRAY_3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, POS_IN_ARRAY> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void POS_IN_ARRAY_10D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<10, POS_IN_ARRAY> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void POS_IN_ARRAY_20D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<20, POS_IN_ARRAY> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void IS_IN_RANGE_3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, IS_IN_RANGE> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void IS_IN_RANGE_10D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<10, IS_IN_RANGE> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void IS_IN_RANGE_20D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<20, IS_IN_RANGE> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void CALC_LIMITS_3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, CALC_LIMITS> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void CALC_LIMITS_10D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<10, CALC_LIMITS> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void CALC_LIMITS_20D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<20, CALC_LIMITS> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

// index type, scenario name, data_generator, num_entities, function_to_call
BENCHMARK_CAPTURE(POS_IN_ARRAY_3D, MAP, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{10, 1000}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(POS_IN_ARRAY_10D, X, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{10, 1000}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(POS_IN_ARRAY_20D, X, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{10, 1000}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(IS_IN_RANGE_3D, X, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{10, 1000}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(IS_IN_RANGE_10D, X, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{10, 1000}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(IS_IN_RANGE_20D, X, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{10, 1000}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(CALC_LIMITS_3D, X, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{10, 1000}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(CALC_LIMITS_10D, X, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{10, 1000}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(CALC_LIMITS_20D, X, 0.0)
    ->RangeMultiplier(10)
    ->Ranges({{10, 1000}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
