// SPDX-License-Identifier: LGPL-3.0-or-later

#include <benchmark/benchmark.h>

#include "CollisionGeometry.hpp"
#include "buildGeometries.hpp"

template <class... Args>
void bmLineSegmentsInDistanceTo(benchmark::State& state, Args&&... args)
{
    auto args_tuple = std::make_tuple(std::move(args)...);
    auto geometry = std::move(std::get<CollisionGeometry>(args_tuple));

    for(auto _ : state) {
        benchmark::DoNotOptimize(geometry.LineSegmentsInDistanceTo(5., {0, 0}));
        benchmark::ClobberMemory();
    }
}

template <class... Args>
void bmLineSegmentsInApproxDistanceTo(benchmark::State& state, Args&&... args)
{
    auto args_tuple = std::make_tuple(std::move(args)...);
    auto geometry = std::move(std::get<CollisionGeometry>(args_tuple));

    for(auto _ : state) {
        benchmark::DoNotOptimize(geometry.LineSegmentsInApproxDistanceTo({0, 0}));
        benchmark::ClobberMemory();
    }
}

BENCHMARK_CAPTURE(bmLineSegmentsInDistanceTo, large_street_network, buildLargeStreetNetwork());

BENCHMARK_CAPTURE(bmLineSegmentsInDistanceTo, grosser_stern, buildGrosserStern());

BENCHMARK_CAPTURE(
    bmLineSegmentsInApproxDistanceTo,
    large_street_network,
    buildLargeStreetNetwork());

BENCHMARK_CAPTURE(bmLineSegmentsInApproxDistanceTo, grosser_stern, buildGrosserStern());
