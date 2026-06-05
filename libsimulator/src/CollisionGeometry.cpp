// SPDX-License-Identifier: LGPL-3.0-or-later
#include "CollisionGeometry.hpp"

#include "AABB.hpp"
#include "CfgCgal.hpp"
#include "GeometricFunctions.hpp"
#include "LineSegment.hpp"
#include "Point.hpp"

#include <CGAL/Boolean_set_operations_2/oriented_side.h>
#include <CGAL/enum.h>
#include <CGAL/number_utils.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <set>
#include <tuple>
#include <vector>

Cell makeCell(Point p)
{
    return {floor(p.X / CELL_EXTEND) * CELL_EXTEND, floor(p.Y / CELL_EXTEND) * CELL_EXTEND};
}

bool IsN8Adjacent(const Cell& a, const Cell& b)
{
    const auto dx = static_cast<int>(abs(a.X - b.X) / CELL_EXTEND);
    const auto dy = static_cast<int>(abs(a.Y - b.Y) / CELL_EXTEND);
    return !((dx == 0 && dy == 0) || dx > 1 || dy > 1);
}

std::set<Cell> cellsFromLineSegment(LineSegment ls)
{
    const auto firstCell = makeCell(ls.P1);
    const auto lastCell = makeCell(ls.P2);
    if(firstCell == lastCell) {
        return {firstCell};
    }

    if(IsN8Adjacent(firstCell, lastCell)) {
        return {firstCell, lastCell};
    }

    std::set<Cell> cells{firstCell, lastCell};

    const auto toMultiple = [](double x) { return ceil(x / CELL_EXTEND) * CELL_EXTEND; };
    const AABB bounds(ls.P1, ls.P2);
    const auto vec_p1p2 = ls.P2 - ls.P1;
    std::vector<Point> intersections{};
    for(double x_intersect = toMultiple(bounds.XMin); x_intersect <= bounds.XMax;
        x_intersect += CELL_EXTEND) {
        const double fact = (x_intersect - ls.P1.X) / vec_p1p2.X;
        intersections.emplace_back(x_intersect, ls.P1.Y + fact * vec_p1p2.Y);
    }
    for(double y_intersect = toMultiple(bounds.YMin); y_intersect <= bounds.YMax;
        y_intersect += CELL_EXTEND) {
        const double fact = (y_intersect - ls.P1.Y) / vec_p1p2.Y;
        intersections.emplace_back(ls.P1.X + fact * vec_p1p2.X, y_intersect);
    }
    std::sort(std::begin(intersections), std::end(intersections));
    for(size_t index = 1; index < intersections.size(); ++index) {
        cells.insert(makeCell((intersections[index - 1] + intersections[index]) / 2));
    }
    return cells;
}

double dist(LineSegment l, Point p)
{
    return l.DistTo(p);
}

size_t CountLineSegments(const PolyWithHoles& poly)
{
    auto count = poly.outer_boundary().size();
    for(const auto& hole : poly.holes()) {
        count += hole.size();
    }
    return count;
}

Point fromPoint_2(const K::Point_2& p)
{
    return {CGAL::to_double(p.x()), CGAL::to_double(p.y())};
}

void ExtractSegmentsFromPolygon(const Poly& p, std::vector<LineSegment>& segments)
{
    const auto& boundary = p.container();
    for(size_t index = 1; index < boundary.size(); ++index) {
        segments.emplace_back(fromPoint_2(boundary[index - 1]), fromPoint_2(boundary[index]));
    }
    segments.emplace_back(fromPoint_2(boundary.back()), fromPoint_2(boundary.front()));
}

CollisionGeometry::CollisionGeometry(PolyWithHoles accessibleArea)
    : accessibleAreaPolygon(accessibleArea)
{
    segments.reserve(CountLineSegments(accessibleArea));
    ExtractSegmentsFromPolygon(accessibleArea.outer_boundary(), segments);
    for(const auto& hole : accessibleArea.holes()) {
        ExtractSegmentsFromPolygon(hole, segments);
    }

    for(const auto& ls : segments) {
        const auto cells = cellsFromLineSegment(ls);
        for(const auto& cell : cells) {
            grid[cell].insert(ls);
        }

        insertIntoApproximateGrid(ls);
    }

    for(auto& [_, vec] : approximateGrid) {
        vec.shrink_to_fit();
    }

    const auto cvt = [](const auto& c) {
        std::vector<Point> out{};
        out.reserve(c.size());
        std::transform(std::begin(c), std::end(c), std::back_inserter(out), [](auto&& p) {
            return fromPoint_2(p);
        });
        return out;
    };
    std::vector<Point> exterior = cvt(accessibleAreaPolygon.outer_boundary().container());
    std::vector<std::vector<Point>> holes{};
    holes.reserve(accessibleAreaPolygon.holes().size());
    std::transform(
        std::begin(accessibleAreaPolygon.holes()),
        std::end(accessibleAreaPolygon.holes()),
        std::back_inserter(holes),
        [&cvt](auto&& c) { return cvt(c); });
    this->accessibleArea = std::make_tuple(exterior, holes);
}

const std::vector<LineSegment>& CollisionGeometry::LineSegmentsInApproxDistanceTo(Point p) const
{
    const auto cell = makeCell(p);
    if(const auto it = approximateGrid.find(cell); it != approximateGrid.end()) {
        return it->second;
    }
    static const std::vector<LineSegment> empty{};
    return empty;
}

void CollisionGeometry::insertIntoApproximateGrid(const LineSegment& ls)
{
    constexpr double searchRadius = 4.;

    const auto searchExtend = Point(searchRadius, searchRadius);
    const AABB lineSegmentBounds({ls.P1, ls.P2});
    const AABB searchBounds(
        lineSegmentBounds.BottomLeft() - searchExtend, lineSegmentBounds.TopRight() + searchExtend);

    auto cellBottomLeft = makeCell(searchBounds.BottomLeft());
    auto cellTopRight = makeCell(searchBounds.TopRight());

    for(double x = cellBottomLeft.X; x <= cellTopRight.X; x += CELL_EXTEND) {
        for(double y = cellBottomLeft.Y; y <= cellTopRight.Y; y += CELL_EXTEND) {
            const auto cell = makeCell({x, y});

            const AABB bbWithSearchRadius(
                {cell.X - searchRadius, cell.Y - searchRadius},
                {cell.X + searchRadius + CELL_EXTEND, cell.Y + searchRadius + CELL_EXTEND});

            if(bbWithSearchRadius.Intersects(ls)) {
                auto& vec = approximateGrid[cell];
                vec.push_back(ls);
            }
        }
    }
}

CollisionGeometry::LineSegmentRange
CollisionGeometry::LineSegmentsInDistanceTo(double distance, Point p) const
{
    return LineSegmentRange{
        DistanceQueryIterator<LineSegment>{distance, p, segments.cbegin(), segments.cend()},
        DistanceQueryIterator<LineSegment>{distance, p, segments.cend(), segments.cend()}};
}

bool CollisionGeometry::IntersectsAny(const LineSegment& linesegment) const
{
    const auto cellsToQuery = cellsFromLineSegment(linesegment);
    for(const auto& cell : cellsToQuery) {
        const auto iter = grid.find(cell);
        if(iter == std::end(grid)) {
            continue;
        }
        if(std::find_if(
               iter->second.cbegin(), iter->second.cend(), [&linesegment](const auto candidate) {
                   return intersects(linesegment, candidate);
               }) != iter->second.end()) {
            return true;
        }
    }
    return false;
}

bool CollisionGeometry::InsideGeometry(Point p) const
{
    return CGAL::oriented_side(K::Point_2(p.X, p.Y), accessibleAreaPolygon) !=
           CGAL::ON_NEGATIVE_SIDE;
}

const std::tuple<std::vector<Point>, std::vector<std::vector<Point>>>&
CollisionGeometry::AccessibleArea() const
{
    return accessibleArea;
}
