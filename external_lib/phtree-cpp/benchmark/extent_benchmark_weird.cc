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

/*
 * Benchmark for iterating over all entries while using a filter.
 *
 * TODO
 * This benchmarks shows some weird behaviour, see below.
 * It should probably be removed at some point.
 */

template <dimension_t DIM>
class IndexBenchmark {
  public:
    IndexBenchmark(
        benchmark::State& state, TestGenerator data_type, int num_entities, int type = 0);

    void Benchmark(benchmark::State& state);

  private:
    void SetupWorld(benchmark::State& state);
    void QueryWorld(benchmark::State& state);

    const TestGenerator data_type_;
    const size_t num_entities_;

    PhTree<DIM, int> tree_;
    std::default_random_engine random_engine_;
    std::uniform_int_distribution<> cube_distribution_;
    std::vector<PhPoint<DIM>> points_;
    int type_;
};

template <dimension_t DIM>
IndexBenchmark<DIM>::IndexBenchmark(
    benchmark::State& state, TestGenerator data_type, int num_entities, int type)
: data_type_{data_type}
, num_entities_(num_entities)
, random_engine_{1}
, cube_distribution_{0, GLOBAL_MAX}
, points_(num_entities)
, type_{type} {
    logging::SetupDefaultLogging();
    SetupWorld(state);
}

template <dimension_t DIM>
void IndexBenchmark<DIM>::Benchmark(benchmark::State& state) {
    for (auto _ : state) {
        QueryWorld(state);
    }
}

template <dimension_t DIM>
void IndexBenchmark<DIM>::SetupWorld(benchmark::State& state) {
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

template <dimension_t DIM, typename KEY = PhPoint<DIM>, typename SCALAR = scalar_64_t>
class FilterBoxIntersection {
  public:
    FilterBoxIntersection(const PhPoint<DIM>& min_include, const PhPoint<DIM>& max_include)
    : min_include_bits{min_include}, max_include_bits{max_include} {};

    void set(const PhPointD<DIM>& minExclude, const PhPointD<DIM>& maxExclude) {
        min_include_bits = PRE(minExclude);
        max_include_bits = PRE(maxExclude);
    }

    [[nodiscard]] bool IsEntryValid(const PhPoint<DIM>& key, const int&) const {
        for (int i = 0; i < DIM; ++i) {
            if (key[i] < min_include_bits[i] || key[i] > max_include_bits[i]) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool IsNodeValid(const PhPoint<DIM>& prefix, int bits_to_ignore) const {
        // skip this for root node (bitsToIgnore == 64)
        if (bits_to_ignore >= (detail::MAX_BIT_WIDTH<SCALAR> - 1)) {
            return true;
        }
        detail::bit_mask_t<SCALAR> mask_min = detail::MAX_MASK<SCALAR> << bits_to_ignore;
        detail::bit_mask_t<SCALAR> mask_max = ~mask_min;

        for (size_t i = 0; i < prefix.size(); ++i) {
            if ((prefix[i] | mask_max) < min_include_bits[i] ||
                (prefix[i] & mask_min) > max_include_bits[i]) {
                return false;
            }
        }
        return true;
    }

  private:
    const PhPoint<DIM> min_include_bits;
    const PhPoint<DIM> max_include_bits;
};

template <dimension_t DIM, typename KEY = PhPoint<DIM>>
class FilterTrue {
  public:
    FilterTrue(const PhPoint<DIM>& minInclude, const PhPoint<DIM>& maxInclude)
    : minIncludeBits{minInclude}, maxIncludeBits{maxInclude} {};

    void set(const PhPointD<DIM>& minExclude, const PhPointD<DIM>& maxExclude) {
        minIncludeBits = PRE(minExclude);
        maxIncludeBits = PRE(maxExclude);
    }

    [[nodiscard]] bool IsEntryValid(const PhPoint<DIM>&, const int&) const {
        return true;
    }

    [[nodiscard]] bool IsNodeValid(const PhPoint<DIM>&, int) const {
        return true;
    }

  private:
    const PhPoint<DIM> minIncludeBits;
    const PhPoint<DIM> maxIncludeBits;
};

template <dimension_t DIM, typename KEY = PhPoint<DIM>>
class FilterTrue2 {
  public:
    FilterTrue2() : minIncludeBits{}, maxIncludeBits{} {};

    [[nodiscard]] bool IsEntryValid(const PhPoint<DIM>&, const int&) const {
        return true;
    }

    [[nodiscard]] bool IsNodeValid(const PhPoint<DIM>&, int) const {
        return true;
    }

  private:
    const PhPoint<DIM> minIncludeBits;
    const PhPoint<DIM> maxIncludeBits;
};

template <dimension_t DIM, typename T>
struct FilterTrue3 {
    [[nodiscard]] constexpr bool IsEntryValid(const PhPoint<DIM>&, const T&) const {
        return true;
    }

    [[nodiscard]] constexpr bool IsNodeValid(const PhPoint<DIM>&, int) const {
        return true;
    }
};

template <dimension_t DIM>
void IndexBenchmark<DIM>::QueryWorld(benchmark::State& state) {
    int n = 0;
    // TODO This is all really weird.
    //   reenabling one of the following filters has the follwing effects:
    //   1) Some of the filters in the first branch will affect performance of the
    //      second branch (?!?!?!)
    //   2) Performance is often different from the second branch if the the filters are
    //      logically the same.
    //   Differences are usually between 5% and 15%, but confidence is pretty high
    //   if the tests at thye end of the file are enabled (notice the somewhat irregular
    //   pattern of the tests that will find itself clearly in the results:
    //   Order: 0 1 0 0 1 1 0 1
    //
    //  Some observations:
    //  - Whichever test if in the 'if' part is hardly slowed down, but the 'else'
    //    part is clearly slowed down.
    //  - Compiling with  -falign-functions=32 or -falign-functions=64 did not help
    if (type_ == 0) {
        // PhPoint<DIM> min{-GLOBAL_MAX, -GLOBAL_MAX, -GLOBAL_MAX};
        // PhPoint<DIM> max{GLOBAL_MAX, GLOBAL_MAX, GLOBAL_MAX};
        // FilterAABB<DIM, int> filter(min, max);
        // FilterBoxIntersection<DIM> filter(min, max);
        // FilterNoOp<DIM, int> filter;
        // FilterTrue<DIM> filter(min, max);
        // FilterTrue2<DIM> filter;
        FilterTrue3<DIM, int> filter;
        auto q = tree_.begin(filter);

        //       auto q = tree_.begin();
        while (q != tree_.end()) {
            // just read the entry
            ++q;
            ++n;
        }
    } else {
        auto q = tree_.begin();
        while (q != tree_.end()) {
            // just read the entry
            ++q;
            ++n;
        }
    }

    state.counters["total_result_count"] += n;
    state.counters["query_rate"] += 1;
    state.counters["result_rate"] += n;
    state.counters["avg_result_count"] += n;
}

}  // namespace

template <typename... Arguments>
void PhTree3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

// index type, scenario name, data_generator, num_entities, filter selector
// PhTree 3D CUBE
BENCHMARK_CAPTURE(PhTree3D, EXT_CU_1K, TestGenerator::CUBE, 1000, 0)->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, EXT_CU_1K, TestGenerator::CUBE, 1000, 1)->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, EXT_CU_1K, TestGenerator::CUBE, 1000, 0)->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, EXT_CU_1K, TestGenerator::CUBE, 1000, 0)->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, EXT_CU_1K, TestGenerator::CUBE, 1000, 1)->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, EXT_CU_1K, TestGenerator::CUBE, 1000, 1)->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, EXT_CU_1K, TestGenerator::CUBE, 1000, 0)->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(PhTree3D, EXT_CU_1K, TestGenerator::CUBE, 1000, 1)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
