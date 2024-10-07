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
 * Benchmark for updating the position of entries in multi-map implementations.
 */
namespace {

constexpr size_t UPDATES_PER_ROUND = 1000;
std::vector<double> MOVE_DISTANCE = {0, 1.0, 10};

const double GLOBAL_MAX = 10000;

enum Scenario { ERASE_EMPLACE, MM_BPT_RELOCATE, MM_SET_RELOCATE, MM_SET_RELOCATE_IF };

using payload_t = scalar_64_t;

using BucketType = std::set<payload_t>;

template <dimension_t DIM>
using PointType = PhPointD<DIM>;

template <Scenario SCENARIO, dimension_t DIM>
using CONVERTER = ConverterIEEE<DIM>;

template <Scenario SCENARIO, dimension_t DIM>
using TestMap = typename std::conditional_t<
    SCENARIO == ERASE_EMPLACE,
    PhTreeD<DIM, BucketType, CONVERTER<SCENARIO, DIM>>,
    typename std::conditional_t<
        SCENARIO == MM_BPT_RELOCATE,
        PhTreeMultiMapD<DIM, payload_t, CONVERTER<SCENARIO, DIM>, b_plus_tree_hash_set<payload_t>>,
        PhTreeMultiMapD<DIM, payload_t, CONVERTER<SCENARIO, DIM>, std::set<payload_t>>>>;

template <dimension_t DIM>
struct UpdateOp {
    payload_t id_;
    PointType<DIM> old_;
    PointType<DIM> new_;
};

template <dimension_t DIM, Scenario SCENARIO>
class IndexBenchmark {
  public:
    explicit IndexBenchmark(
        benchmark::State& state,
        size_t updates_per_round = UPDATES_PER_ROUND,
        std::vector<double> move_distance = MOVE_DISTANCE);

    void Benchmark(benchmark::State& state);

  private:
    void SetupWorld(benchmark::State& state);
    void BuildUpdates();
    void UpdateWorld(benchmark::State& state);

    const TestGenerator data_type_;
    const size_t num_entities_;
    const size_t updates_per_round_;
    const std::vector<double> move_distance_;

    TestMap<SCENARIO, DIM> tree_;
    std::vector<PointType<DIM>> points_;
    std::vector<UpdateOp<DIM>> updates_;
    std::default_random_engine random_engine_;
    std::uniform_int_distribution<> entity_id_distribution_;
};

template <dimension_t DIM, Scenario SCENARIO>
IndexBenchmark<DIM, SCENARIO>::IndexBenchmark(
    benchmark::State& state, size_t updates_per_round, std::vector<double> move_distance)
: data_type_{static_cast<TestGenerator>(state.range(1))}
, num_entities_(state.range(0))
, updates_per_round_(updates_per_round)
, move_distance_(std::move(move_distance))
, points_(num_entities_)
, updates_(updates_per_round)
, random_engine_{0}
, entity_id_distribution_{0, static_cast<int>(num_entities_ - 1)} {
    logging::SetupDefaultLogging();
    SetupWorld(state);
}

template <dimension_t DIM, Scenario SCENARIO>
void IndexBenchmark<DIM, SCENARIO>::Benchmark(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        BuildUpdates();
        state.ResumeTiming();

        UpdateWorld(state);
    }
}

template <dimension_t DIM>
void InsertEntry(
    TestMap<Scenario::ERASE_EMPLACE, DIM>& tree, const PointType<DIM>& point, payload_t data) {
    BucketType& bucket = tree.emplace(point).first;
    bucket.emplace(data);
}

template <dimension_t DIM>
void InsertEntry(
    TestMap<Scenario::MM_BPT_RELOCATE, DIM>& tree, const PointType<DIM>& point, payload_t data) {
    tree.emplace(point, data);
}

template <dimension_t DIM>
void InsertEntry(
    TestMap<Scenario::MM_SET_RELOCATE, DIM>& tree, const PointType<DIM>& point, payload_t data) {
    tree.emplace(point, data);
}

template <dimension_t DIM, Scenario SCENARIO>
typename std::enable_if<SCENARIO == Scenario::ERASE_EMPLACE, size_t>::type UpdateEntry(
    TestMap<SCENARIO, DIM>& tree, std::vector<UpdateOp<DIM>>& updates) {
    size_t n = 0;
    for (auto& update : updates) {
        auto pair_with_bucket = tree.emplace(update.new_);
        auto result_of_bucket_emplace = pair_with_bucket.first.emplace(update.id_);
        if (!result_of_bucket_emplace.second) {
            // emplace failed -> entry already exists. We are done!
            ++n;
            continue;
        }

        // Entry is already inserted, now remove old entry.
        auto iter_old_bucket = tree.find(update.old_);
        assert(iter_old_bucket != tree.end());
        bool success = iter_old_bucket->erase(update.id_);
        if (iter_old_bucket->empty()) {
            success &= tree.erase(iter_old_bucket) != 0;
        }
        n += success;
    }
    return n;
}

template <dimension_t DIM, Scenario SCENARIO>
typename std::enable_if<
    SCENARIO == Scenario::MM_BPT_RELOCATE || SCENARIO == Scenario::MM_SET_RELOCATE,
    size_t>::type
UpdateEntry(TestMap<SCENARIO, DIM>& tree, std::vector<UpdateOp<DIM>>& updates) {
    size_t n = 0;
    for (auto& update : updates) {
        n += tree.relocate(update.old_, update.new_, update.id_);
    }
    return n;
}

template <dimension_t DIM, Scenario SCENARIO>
typename std::enable_if<SCENARIO == Scenario::MM_SET_RELOCATE_IF, size_t>::type UpdateEntry(
    TestMap<SCENARIO, DIM>& tree, std::vector<UpdateOp<DIM>>& updates) {
    size_t n = 0;
    for (auto& update : updates) {
        n += tree.relocate_if(
            update.old_, update.new_, [&update](const payload_t& v) { return v == update.id_; });
    }
    return n;
}

template <dimension_t DIM, Scenario SCENARIO>
void IndexBenchmark<DIM, SCENARIO>::SetupWorld(benchmark::State& state) {
    logging::info("Setting up world with {} entities and {} dimensions.", num_entities_, DIM);
    // create data with about 10% duplicate coordinates
    CreatePointData<DIM>(points_, data_type_, num_entities_, 0, GLOBAL_MAX, 0.1);
    for (size_t i = 0; i < num_entities_; ++i) {
        InsertEntry(tree_, points_[i], i);
    }

    state.counters["total_upd_count"] = benchmark::Counter(0);
    state.counters["update_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);
    logging::info("World setup complete.");
}

template <dimension_t DIM, Scenario SCENARIO>
void IndexBenchmark<DIM, SCENARIO>::BuildUpdates() {
    size_t move_id = 0;
    for (auto& update : updates_) {
        auto point_id = entity_id_distribution_(random_engine_);
        update.id_ = point_id;
        update.old_ = points_[point_id];
        for (dimension_t d = 0; d < DIM; ++d) {
            update.new_[d] = update.old_[d] + move_distance_[move_id];
        }
        // update reference data
        points_[point_id] = update.new_;

        move_id = (move_id + 1) % move_distance_.size();
    }
}

template <dimension_t DIM, Scenario SCENARIO>
void IndexBenchmark<DIM, SCENARIO>::UpdateWorld(benchmark::State& state) {
    size_t initial_tree_size = tree_.size();
    size_t n = 0;
    n = UpdateEntry<DIM, SCENARIO>(tree_, updates_);
    if (n != updates_.size()) {
        logging::error("Invalid update count: {}/{}", updates_.size(), n);
    }

    // For normal indexes we expect num_entities==size(), but the PhTree<Map<...>> index has
    // size() as low as (num_entities-duplicates).
    if (tree_.size() > num_entities_ || tree_.size() + updates_per_round_ < initial_tree_size) {
        logging::error("Invalid index size after update: {}/{}", tree_.size(), num_entities_);
    }

    state.counters["total_upd_count"] += updates_per_round_;
    state.counters["update_rate"] += updates_per_round_;
}

}  // namespace

template <typename... Arguments>
void PhTreeMMRelocateIfStdSet3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, Scenario::MM_SET_RELOCATE_IF> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void PhTreeMMRelocateBpt3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, Scenario::MM_BPT_RELOCATE> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void PhTreeMMRelocateStdSet3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, Scenario::MM_SET_RELOCATE> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void PhTreeMMEraseEmplace3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, Scenario::ERASE_EMPLACE> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

// index type, scenario name, data_type, num_entities, updates_per_round, move_distance
// PhTreeMultiMap
BENCHMARK_CAPTURE(PhTreeMMRelocateIfStdSet3D, UPDATE_1000, UPDATES_PER_ROUND)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

// PhTreeMultiMap with b_plus_tree_hash_map
BENCHMARK_CAPTURE(PhTreeMMRelocateBpt3D, UPDATE_1000, UPDATES_PER_ROUND)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

// PhTreeMultiMap with std::set
BENCHMARK_CAPTURE(PhTreeMMRelocateStdSet3D, UPDATE_1000, UPDATES_PER_ROUND)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

// PhTree<std::map> (manual bucket handling)
BENCHMARK_CAPTURE(PhTreeMMEraseEmplace3D, UPDATE_1000, UPDATES_PER_ROUND)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
