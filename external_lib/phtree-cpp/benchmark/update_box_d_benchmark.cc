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

constexpr size_t UPDATES_PER_ROUND = 1000;
constexpr double MOVE_DISTANCE = 10;

const double GLOBAL_MAX = 10000;
const double BOX_LEN = 10;

enum UpdateType { RELOCATE, ERASE_BY_KEY };

template <dimension_t DIM>
using BoxType = PhBoxD<DIM>;

template <dimension_t DIM>
using TreeType = PhTreeBoxD<DIM, size_t>;

template <dimension_t DIM>
struct UpdateOp {
    size_t id_;
    BoxType<DIM> old_;
    BoxType<DIM> new_;
};

/*
 * Benchmark for updating the position of entries.
 */
template <dimension_t DIM, UpdateType UPDATE_TYPE>
class IndexBenchmark {
  public:
    IndexBenchmark(
        benchmark::State& state,
        size_t updates_per_round = UPDATES_PER_ROUND,
        double move_distance = MOVE_DISTANCE);

    void Benchmark(benchmark::State& state);

  private:
    void SetupWorld(benchmark::State& state);
    void BuildUpdates();
    void UpdateWorld(benchmark::State& state);

    const TestGenerator data_type_;
    const size_t num_entities_;
    const size_t updates_per_round_;
    const double move_distance_;

    TreeType<DIM> tree_;
    std::vector<BoxType<DIM>> boxes_;
    std::vector<UpdateOp<DIM>> updates_;
    std::default_random_engine random_engine_;
    std::uniform_int_distribution<> entity_id_distribution_;
};

template <dimension_t DIM, UpdateType UPDATE_TYPE>
IndexBenchmark<DIM, UPDATE_TYPE>::IndexBenchmark(
    benchmark::State& state, size_t updates_per_round, double move_distance)
: data_type_{static_cast<TestGenerator>(state.range(1))}
, num_entities_(state.range(0))
, updates_per_round_(updates_per_round)
, move_distance_(move_distance)
, boxes_(num_entities_)
, updates_(updates_per_round)
, random_engine_{0}
, entity_id_distribution_{0, static_cast<int>(num_entities_ - 1)} {
    logging::SetupDefaultLogging();
    SetupWorld(state);
}

template <dimension_t DIM, UpdateType UPDATE_TYPE>
void IndexBenchmark<DIM, UPDATE_TYPE>::Benchmark(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        BuildUpdates();
        state.ResumeTiming();

        UpdateWorld(state);
    }
}

template <dimension_t DIM, UpdateType UPDATE_TYPE>
void IndexBenchmark<DIM, UPDATE_TYPE>::SetupWorld(benchmark::State& state) {
    logging::info("Setting up world with {} entities and {} dimensions.", num_entities_, DIM);
    CreateBoxData<DIM>(boxes_, data_type_, num_entities_, 0, GLOBAL_MAX, BOX_LEN);
    for (size_t i = 0; i < num_entities_; ++i) {
        tree_.emplace(boxes_[i], (int)i);
    }

    state.counters["total_upd_count"] = benchmark::Counter(0);
    state.counters["update_rate"] = benchmark::Counter(0, benchmark::Counter::kIsRate);
    logging::info("World setup complete.");
}

template <dimension_t DIM, UpdateType UPDATE_TYPE>
void IndexBenchmark<DIM, UPDATE_TYPE>::BuildUpdates() {
    for (auto& update : updates_) {
        int box_id = entity_id_distribution_(random_engine_);
        update.id_ = box_id;
        update.old_ = boxes_[box_id];
        for (dimension_t d = 0; d < DIM; ++d) {
            update.new_.min()[d] = update.old_.min()[d] + move_distance_;
            update.new_.max()[d] = update.old_.max()[d] + move_distance_;
        }
        // update reference data
        boxes_[box_id] = update.new_;
    }
}

template <dimension_t DIM>
size_t UpdateByRelocate(TreeType<DIM>& tree, std::vector<UpdateOp<DIM>>& updates) {
    size_t n = 0;
    for (auto& update : updates) {
        n += tree.relocate(update.old_, update.new_);
    }
    return n;
}

template <dimension_t DIM>
size_t UpdateByKey(TreeType<DIM>& tree, std::vector<UpdateOp<DIM>>& updates) {
    size_t n = 0;
    for (auto& update : updates) {
        size_t result_erase = tree.erase(update.old_);
        auto result_emplace = tree.emplace(update.new_, update.id_);
        n += result_erase == 1 && result_emplace.second;
    }
    return n;
}

template <dimension_t DIM, UpdateType UPDATE_TYPE>
void IndexBenchmark<DIM, UPDATE_TYPE>::UpdateWorld(benchmark::State& state) {
    size_t initial_tree_size = tree_.size();
    size_t n = 0;
    switch (UPDATE_TYPE) {
    case UpdateType::ERASE_BY_KEY:
        n = UpdateByKey(tree_, updates_);
        break;
    case UpdateType::RELOCATE:
        n = UpdateByRelocate(tree_, updates_);
        break;
    }

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
void PhTreeRelocate3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, UpdateType::RELOCATE> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

template <typename... Arguments>
void PhTree3D(benchmark::State& state, Arguments&&... arguments) {
    IndexBenchmark<3, UpdateType::ERASE_BY_KEY> benchmark{state, arguments...};
    benchmark.Benchmark(state);
}

// index type, scenario name, data_type, num_entities, updates_per_round, move_distance
// PhTree with relocate()
BENCHMARK_CAPTURE(PhTreeRelocate3D, UPDATE_1000, UPDATES_PER_ROUND)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

// PhTree with relocate()
BENCHMARK_CAPTURE(PhTree3D, UPDATE_1000, UPDATES_PER_ROUND)
    ->RangeMultiplier(10)
    ->Ranges({{1000, 1000 * 1000}, {TestGenerator::CUBE, TestGenerator::CLUSTER}})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
