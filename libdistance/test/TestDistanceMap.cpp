// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include <common_types.hpp>
#include <grid.hpp>
#include <gtest/gtest.h>
#include <raster_map.hpp>

TEST(Grid, CanConstruct)
{
    using GridType = distance::Grid<double>;
    const int width = 8;
    const int height = 12;
    const double expected_value{};
    auto g = GridType(width, height, expected_value);
    ASSERT_EQ(g.Width(), width);
    ASSERT_EQ(g.Height(), height);
    ASSERT_EQ(g.Size(), width * height);
    for(auto&& v : g.Data()) {
        ASSERT_EQ(v, expected_value);
    }
}

class MarkCellsInGrid : public ::testing::Test
{
    using GridType = distance::Grid<double>;

protected:
    uint64_t height{10};
    uint64_t width{10};
    GridType g;
    static constexpr double q_nan{std::numeric_limits<double>::quiet_NaN()};

public:
    MarkCellsInGrid() : g(width, height, q_nan)
    {
        for(auto&& v : g.Data()) {
            EXPECT_TRUE(std::isnan(v));
        }
    }
};

TEST_F(MarkCellsInGrid, MarkPointInside)
{
    g.At({0, 0}) = 1;
    ASSERT_EQ(g.At({0, 0}), 1);
}

TEST(RasterMap, CanConstruct2)
{
    distance::AABB<double> bounds{{-1.0, -1.0}, {1.0, 1.0}};
    distance::RasterMap<double, int> map{bounds, 0.3};
    distance::Point<double> p{-1.0 + 0.15, 1.0 - 0.15};

    for(int x = 0; x < 7; ++x) {
        for(int y = 0; y < 7; ++y) {
            map.At({p.x + 0.3 * x, p.y - 0.3 * y}) = x + y * 7;
        }
    }

    for(int x = 0; x < 7; ++x) {
        for(int y = 0; y < 7; ++y) {
            ASSERT_EQ(map.At({p.x + 0.3 * x, p.y - 0.3 * y}), x + y * 7);
        }
    }
}
