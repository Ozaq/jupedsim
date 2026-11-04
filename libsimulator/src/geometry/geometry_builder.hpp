// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "cfg_cgal.hpp"
#include "geometry/geometry.hpp"
#include "line_segment.hpp"
#include "point.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <fmt/ranges.h>

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

class Geometry;

class GeometryBuilder
{
public:
    using Ring = std::vector<Point>;
    struct Polygon {
        Ring boundary;
        std::vector<Ring> holes;
    };

    size_t add_region(Polygon polygon, double height);
    size_t connect_regions(size_t from_region, LineSegment from, size_t to_region, LineSegment to);

    using RegionGraph2D = Geometry::RegionGraph2D;
    std::unique_ptr<RegionGraph2D> create_region_graph_2d() const;

    /// Creates the geometry from the regions and connections added so far. The builder is
    /// left unchanged, so it can be extended and built again.
    std::unique_ptr<Geometry> build() const;

private:
    /// Minimum clearance around each vertex in meters.
    static constexpr double min_clearance{0.025};
    std::vector<epick::Point3D> _global_vertices;

    struct Region {
        // Store boundary (index 0) + holes as vector of indices into globalVertices
        std::vector<std::vector<size_t>> polygons;
        /// Empty for connectors, which are inclined instead of flat.
        std::optional<double> height;
        /// Polygons projected to x/y.
        epick::PolyWithHoles poly_with_holes;

        /// Connectors may not be connected again.
        bool is_connectable() const { return height.has_value(); }
    };

    /// Global vertex IDs of the shared edge of connected regions.
    using Seam = std::array<size_t, 2>;

    using RegionGraph =
        boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Region, Seam>;
    RegionGraph _region_graph{};

    std::array<size_t, 2> find_edge(size_t region_id, const LineSegment& edge) const;

    /// Check whether floors at specified height overlap. Throws in case of error.
    void validate_floor_overlap(const epick::PolyWithHoles& poly_with_holes, double height) const;

    /// Checks the input as a whole before the geometry is created. Throws SimulationError.
    void validate() const;
};

/// Converts a Polygon into a CGAL polygon with holes.
/// Copies the rings verbatim: they are neither checked nor reoriented, so the result only has the
/// winding CGAL expects if @p polygon has it.
template <typename Kernel>
struct Convert<GeometryBuilder::Polygon, CGAL::Polygon_with_holes_2<Kernel>> {
    static CGAL::Polygon_with_holes_2<Kernel> from(const GeometryBuilder::Polygon& polygon)
    {
        const auto convert_ring = [](const GeometryBuilder::Ring& ring) {
            CGAL::Polygon_2<Kernel> converted{};
            for(const Point& point : ring) {
                converted.push_back(into<CGAL::Point_2<Kernel>>(point));
            }
            return converted;
        };

        std::vector<CGAL::Polygon_2<Kernel>> holes{};
        holes.reserve(polygon.holes.size());
        for(const auto& hole : polygon.holes) {
            holes.emplace_back(convert_ring(hole));
        }
        return CGAL::Polygon_with_holes_2<Kernel>(
            convert_ring(polygon.boundary), std::begin(holes), std::end(holes));
    }
};

template <>
struct fmt::formatter<GeometryBuilder::Polygon> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const GeometryBuilder::Polygon& p, FormatContext& ctx) const
    {
        return fmt::format_to(ctx.out(), "({}, {})", p.boundary, p.holes);
    }
};
