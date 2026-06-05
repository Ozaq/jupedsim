// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Polygon.hpp"

#include "Point.hpp"
#include "SimulationError.hpp"

#include <CGAL/enum.h>
#include <CGAL/number_utils.h>

#include <algorithm>
#include <tuple>
#include <vector>

Polygon::Polygon(const std::vector<Point>& points)
{
    if(points.size() < 3) {
        throw SimulationError("Polygon must have at least 3 points");
    }
    polygon.resize(points.size());
    std::transform(std::begin(points), std::end(points), polygon.begin(), [](const auto& p) {
        return PolygonType::Point_2{p.X, p.Y};
    });

    if(!polygon.is_simple()) {
        throw SimulationError("Polygon is not simple");
    }

    switch(polygon.orientation()) {
        case CGAL::Orientation::COLLINEAR:
            throw SimulationError("Polygon may not be collinear.");
        case CGAL::Orientation::CLOCKWISE:
            polygon.reverse_orientation();
        case CGAL::Orientation::COUNTERCLOCKWISE:
            break;
    }
}

bool Polygon::IsConvex() const
{
    return polygon.is_convex();
}

bool Polygon::IsInside(Point p) const
{
    const auto side = polygon.bounded_side(PolygonType::Point_2{p.X, p.Y});
    return side != CGAL::Bounded_side::ON_UNBOUNDED_SIDE;
}

Point Polygon::Centroid() const
{
    Point sum{};
    std::for_each(polygon.begin(), polygon.end(), [&sum](const auto& p) {
        sum += Point(CGAL::to_double(p.x()), CGAL::to_double(p.y()));
    });
    return sum / static_cast<double>(polygon.size());
}

std::tuple<Point, double> Polygon::ContainingCircle() const
{
    const auto center = Centroid();
    auto distance = 0.0;
    std::for_each(std::begin(polygon), std::end(polygon), [&distance, center](const auto& p) {
        const Point pt(CGAL::to_double(p.x()), CGAL::to_double(p.y()));
        distance = std::max(distance, (center - pt).Norm());
    });
    return {center, distance};
}
