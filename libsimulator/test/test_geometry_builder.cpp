// SPDX-License-Identifier: LGPL-3.0-or-later
#include "cfg_cgal.hpp"
#include "geometry/geometry_builder.hpp"
#include "geometry_fixtures.hpp"
#include "simulation_error.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <vector>

namespace
{
epick::Poly make_poly(const std::vector<epick::Point2D>& points)
{
    return epick::Poly(std::begin(points), std::end(points));
}

/// Checks that the seam (between regions "from" and "to") connects "a" and "b" and that the point
/// "inside" is left to the seam (has to be in region "from").
void expect_seam(
    const GeometryBuilder::RegionGraph2D& graph,
    size_t from,
    size_t to,
    const epick::Point2D& a,
    const epick::Point2D& b,
    const epick::Point2D& inside)
{
    const auto [edge, exists] = boost::edge(from, to, graph);
    ASSERT_TRUE(exists);
    const auto& seam = graph[edge];
    const epick::PolyWithHoles& poly = graph[from];
    ASSERT_LE(seam.ring, poly.number_of_holes());
    const epick::Poly& ring =
        (seam.ring == 0) ? poly.outer_boundary() : poly.holes()[seam.ring - 1];
    ASSERT_LT(seam.index, ring.size());

    const epick::Point2D& source = ring.vertex(seam.index);
    const epick::Point2D& target = ring.vertex((seam.index + 1) % ring.size());
    EXPECT_TRUE(source == a && target == b);
    EXPECT_EQ(CGAL::orientation(source, target, inside), CGAL::LEFT_TURN);
}
} // namespace

TEST(GeometryBuilder, UnconnectedFloors)
{
    GeometryBuilder surface{};
    const auto first = surface.add_region({{{0, 0}, {1, 0}, {1, 1}, {0, 1}}, {}}, 0.0);
    const auto second = surface.add_region({{{0, 0}, {1, 0}, {1, 1}, {0, 1}}, {}}, 3.0);

    const auto graph = surface.create_region_graph_2d();
    EXPECT_EQ(boost::num_vertices(*graph), 2);
    EXPECT_EQ(boost::num_edges(*graph), 0);
    EXPECT_EQ(first, 0);
    EXPECT_EQ(second, 1);
}

TEST(GeometryBuilder, NonFiniteCoordinatesAreRejected)
{
    const GeometryBuilder::Polygon square{{{0, 0}, {1, 0}, {1, 1}, {0, 1}}, {}};
    for(const double bad :
        {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        const GeometryBuilder::Polygon with_bad_corner{{{0, 0}, {bad, 0}, {1, 1}, {0, 1}}, {}};
        GeometryBuilder surface{};
        EXPECT_THROW(surface.add_region(with_bad_corner, 0.0), SimulationError) << bad;
        EXPECT_THROW(surface.add_region(square, bad), SimulationError) << bad;
    }
}

TEST(GeometryBuilder, ClosedRingsAreRejected)
{
    const GeometryBuilder::Ring closed{{0, 0}, {1, 0}, {1, 1}, {0, 0}};
    const GeometryBuilder::Ring outer{{-1, -1}, {2, -1}, {2, 2}, {-1, 2}};
    GeometryBuilder surface{};
    EXPECT_THROW(surface.add_region({closed, {}}, 0.0), SimulationError);
    EXPECT_THROW(surface.add_region({outer, {closed}}, 0.0), SimulationError);
}

template <typename Kernel>
void expect_polygon_converted_verbatim()
{
    using Point2 = CGAL::Point_2<Kernel>;
    using Ring2 = CGAL::Polygon_2<Kernel>;
    const auto ring = [](const std::vector<Point2>& points) {
        return Ring2(std::begin(points), std::end(points));
    };
    // Boundary clockwise and hole counterclockwise: the wrong way around for CGAL.
    const GeometryBuilder::Polygon polygon{
        {{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{2, 2}, {4, 2}, {4, 4}, {2, 4}}}};

    const auto converted = into<CGAL::Polygon_with_holes_2<Kernel>>(polygon);

    EXPECT_EQ(converted.outer_boundary(), ring({{0, 0}, {0, 10}, {10, 10}, {10, 0}}));
    ASSERT_EQ(converted.number_of_holes(), 1);
    EXPECT_EQ(converted.holes()[0], ring({{2, 2}, {4, 2}, {4, 4}, {2, 4}}));
}

TEST(GeometryBuilder, PolygonConvertsVerbatim)
{
    expect_polygon_converted_verbatim<epick::Kernel>();
    expect_polygon_converted_verbatim<epeck::Kernel>();
}

TEST(GeometryBuilder, PolygonRingsHaveCorrectOrientation)
{
    GeometryBuilder surface{};
    const auto region = surface.add_region(
        {{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{2, 2}, {4, 2}, {4, 4}, {2, 4}}}}, 0.0);

    const auto graph = surface.create_region_graph_2d();
    const epick::PolyWithHoles& poly = (*graph)[region];
    // boundary should be counterclockwise, hole clockwise.
    EXPECT_EQ(poly.outer_boundary(), make_poly({{0, 0}, {10, 0}, {10, 10}, {0, 10}}));
    ASSERT_EQ(poly.number_of_holes(), 1);
    EXPECT_EQ(poly.holes()[0], make_poly({{2, 2}, {2, 4}, {4, 4}, {4, 2}}));
}

TEST(GeometryBuilder, CheckStairsPoly)
{
    const auto stair = test_geometries::straight_stair_to_a_landing();
    const auto* graph = stair.geometry->region_graph_2d();
    ASSERT_NE(graph, nullptr);
    ASSERT_EQ(boost::num_vertices(*graph), 3);
    const epick::PolyWithHoles& flight = (*graph)[stair.connector];
    EXPECT_EQ(flight.outer_boundary(), make_poly({{10, 0}, {15, 0}, {15, 8}, {10, 8}}));
    EXPECT_EQ(flight.number_of_holes(), 0);
}

TEST(GeometryBuilder, SeamsOfConnection)
{
    const auto stair = test_geometries::straight_stair_to_a_landing();
    const epick::Point2D on_ground{5, 4};
    const epick::Point2D on_flight{12.5, 4};
    const epick::Point2D on_landing{17.5, 4};

    const auto* graph = stair.geometry->region_graph_2d();
    ASSERT_NE(graph, nullptr);
    EXPECT_EQ(boost::num_edges(*graph), 4); // both directions at each seam
    EXPECT_FALSE(boost::edge(stair.room_1, stair.room_2, *graph).second);
    EXPECT_FALSE(boost::edge(stair.room_2, stair.room_1, *graph).second);

    expect_seam(*graph, stair.room_1, stair.connector, {10, 0}, {10, 8}, on_ground);
    expect_seam(*graph, stair.connector, stair.room_1, {10, 8}, {10, 0}, on_flight);
    expect_seam(*graph, stair.connector, stair.room_2, {15, 0}, {15, 8}, on_flight);
    expect_seam(*graph, stair.room_2, stair.connector, {15, 8}, {15, 0}, on_landing);
}

TEST(GeometryBuilder, SeamOnHoleEdge)
{
    GeometryBuilder surface{};
    const auto ground = surface.add_region(
        {{{0, 0}, {20, 0}, {20, 10}, {0, 10}},
         {{{2, 2}, {18, 2}, {18, 8}, {2, 8}, {2, 7}, {2, 3}}}},
        0.0);
    const auto upper = surface.add_region({{{12, 3}, {17, 3}, {17, 7}, {12, 7}}, {}}, 1.0);
    const auto stairs =
        surface.connect_regions(ground, {{2, 3}, {2, 7}}, upper, {{12, 3}, {12, 7}});

    const auto graph = surface.create_region_graph_2d();
    const auto [edge, exists] = boost::edge(ground, stairs, *graph);
    ASSERT_TRUE(exists);
    EXPECT_EQ((*graph)[edge].ring, 1);
    expect_seam(*graph, ground, stairs, {2, 3}, {2, 7}, {1, 5});
    expect_seam(*graph, stairs, ground, {2, 7}, {2, 3}, {7, 5});
}

TEST(GeometryBuilder, BuildLeavesTheBuilderReusable)
{
    GeometryBuilder builder{};
    const auto ground = builder.add_region({{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, {}}, 0.0);
    const GeometryBuilder& as_const = builder;
    const auto first = as_const.build();
    const auto again = as_const.build();
    EXPECT_EQ(first->region_count(), 1u);
    EXPECT_EQ(again->region_count(), 1u);

    const auto upper = builder.add_region({{{14, 0}, {20, 0}, {20, 10}, {14, 10}}, {}}, 1.0);
    builder.connect_regions(ground, {{10, 0}, {10, 10}}, upper, {{14, 0}, {14, 10}});
    EXPECT_EQ(builder.build()->region_count(), 3u);
    EXPECT_EQ(first->region_count(), 1u);
}
