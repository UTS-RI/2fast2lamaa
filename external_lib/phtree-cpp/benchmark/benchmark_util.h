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

#ifndef PHTREE_BENCHMARK_UTIL_H
#define PHTREE_BENCHMARK_UTIL_H

#include "phtree/common/common.h"
#include <random>
#include <vector>

namespace improbable::phtree::phbenchmark {

namespace {
template <dimension_t DIM>
auto CreateDataCUBE = [](auto& points,
                         size_t num_entities,
                         std::uint32_t seed,
                         double world_minimum,
                         double world_maximum,
                         auto set_coordinate) {
    std::default_random_engine random_engine{seed};
    std::uniform_real_distribution<> distribution(world_minimum, world_maximum);
    for (size_t i = 0; i < num_entities; ++i) {
        auto& p = points[i];
        for (dimension_t d = 0; d < DIM; ++d) {
            set_coordinate(p, d, distribution(random_engine));
        }
    }
};

template <dimension_t DIM>
auto CreateDataCLUSTER = [](auto& points,
                            size_t num_entities,
                            std::uint32_t seed,
                            double world_mininum,
                            double world_maximum,
                            auto set_coordinate) {
    std::default_random_engine random_engine{seed};
    std::uniform_real_distribution<> distribution(world_mininum, world_maximum);
    // SIGMA = 0.0001
    std::normal_distribution<> gauss_distribution(0, 0.0001);
    const int NUM_PT_PER_CLUSTER = 100;
    // 1000 points per cluster, minimum is 1 cluster.
    size_t num_cluster = std::max(1, (int)(num_entities / NUM_PT_PER_CLUSTER));
    const double world_length = world_maximum - world_mininum;

    // loop over clusters
    PhPointD<DIM> cp;  // center point of cluster
    size_t id = 0;
    for (size_t c = 0; c < num_cluster; ++c) {
        for (dimension_t d = 0; d < DIM; ++d) {
            cp[d] = distribution(random_engine);
        }
        for (size_t i = 0; i < NUM_PT_PER_CLUSTER; ++i) {
            auto& p = points[id++];
            // int ii = (c * N_C + i) * DIM;
            for (dimension_t d = 0; d < DIM; ++d) {
                // double x = (R.nextGaussian() - 0.5) * GAUSS_SIGMA;  // confine to small rectangle
                double x = gauss_distribution(random_engine);
                x *= world_length + world_mininum;  // stretch if domain>1.0
                x += cp[d];                         // offset of cluster
                set_coordinate(p, d, x);
            }
        }
    }
};

auto CreateDuplicates =
    [](auto& points, int num_unique_entries, size_t num_total_entities, std::uint32_t seed) {
        std::default_random_engine random_engine{seed};
        std::uniform_int_distribution<> distribution(0, num_unique_entries);
        for (size_t i = num_unique_entries; i < num_total_entities; ++i) {
            // copy some random other point or box
            points[i] = points[distribution(random_engine)];
        }
    };
}  // namespace

enum TestGenerator { CUBE = 4, CLUSTER = 7 };

template <dimension_t DIM>
auto CreatePointDataMinMax = [](auto& points,
                                TestGenerator test_generator,
                                size_t num_entities,
                                int seed,
                                double world_minimum,
                                double world_maximum,
                                double fraction_of_duplicates) {
    auto set_coordinate_lambda = [](auto& p, dimension_t dim, auto value) {
        p[dim] = static_cast<typename std::remove_reference_t<decltype(p[0])>>(value);
    };
    // Create at least 1 unique point
    // Note that the following point generator is likely, but not guaranteed, to created unique
    // points.
    int num_unique_entries =
        static_cast<int>(1 + static_cast<double>(num_entities - 1) * (1. - fraction_of_duplicates));
    points.reserve(num_entities);
    switch (test_generator) {
    case CUBE:
        CreateDataCUBE<DIM>(
            points, num_unique_entries, seed, world_minimum, world_maximum, set_coordinate_lambda);
        break;
    case CLUSTER:
        CreateDataCLUSTER<DIM>(
            points, num_unique_entries, seed, world_minimum, world_maximum, set_coordinate_lambda);
        break;
    default:
        assert(false);
    }

    // Create duplicates
    CreateDuplicates(points, num_unique_entries, num_entities, seed);
};

template <dimension_t DIM>
auto CreateBoxDataMinMax = [](auto& points,
                              TestGenerator test_generator,
                              size_t num_entities,
                              int seed,
                              double world_minimum,
                              double world_maximum,
                              double box_length,
                              double fraction_of_duplicates) {
    auto set_coordinate_lambda = [box_length](auto& p, dimension_t dim, auto value) {
        p.min()[dim] = value;
        p.max()[dim] = value + box_length;
    };
    // Create at least 1 unique point
    // Note that the following point generator is likely, but not guaranteed, to created unique
    // points.
    int num_unique_entries =
        static_cast<int>(1 + static_cast<double>(num_entities - 1) * (1. - fraction_of_duplicates));
    points.reserve(num_entities);
    switch (test_generator) {
    case CUBE:
        CreateDataCUBE<DIM>(
            points, num_unique_entries, seed, world_minimum, world_maximum, set_coordinate_lambda);
        break;
    case CLUSTER:
        CreateDataCLUSTER<DIM>(
            points, num_unique_entries, seed, world_minimum, world_maximum, set_coordinate_lambda);
        break;
    default:
        assert(false);
    }

    // Create duplicates
    CreateDuplicates(points, num_unique_entries, num_entities, seed);
};

template <dimension_t DIM>
auto CreatePointData = [](auto& points,
                          TestGenerator test_generator,
                          size_t num_entities,
                          int seed,
                          double world_length,
                          double fraction_of_duplicates = 0.) {
    CreatePointDataMinMax<DIM>(
        points, test_generator, num_entities, seed, 0, world_length, fraction_of_duplicates);
};

template <dimension_t DIM>
auto CreateBoxData = [](auto& points,
                        TestGenerator test_generator,
                        size_t num_entities,
                        int seed,
                        double world_length,
                        double box_length,
                        double fraction_of_duplicates = 0.) {
    CreateBoxDataMinMax<DIM>(
        points,
        test_generator,
        num_entities,
        seed,
        0,
        world_length,
        box_length,
        fraction_of_duplicates);
};

}  // namespace improbable::phtree::phbenchmark

#endif  // PHTREE_BENCHMARK_UTIL_H
