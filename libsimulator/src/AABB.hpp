// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "LineSegment.hpp"
#include "Point.hpp"
#include "SimulationError.hpp"

#include <algorithm>
#include <limits>

struct AABB {
    double XMin{std::numeric_limits<double>::max()};
    double XMax{std::numeric_limits<double>::lowest()};
    double YMin{std::numeric_limits<double>::max()};
    double YMax{std::numeric_limits<double>::lowest()};

    AABB() = default;

    template <typename Container>
    AABB(const Container& container)
    {
        if(std::empty(container)) {
            throw SimulationError("Cannot create a AABB from zero points");
        }
        for(const auto [x, y] : container) {
            XMin = std::min(x, XMin);
            XMax = std::max(x, XMax);
            YMin = std::min(y, YMin);
            YMax = std::max(y, YMax);
        };
    }

    AABB(const Point a, const Point b)
    {
        XMin = std::min(b.X, std::min(a.X, XMin));
        XMax = std::max(b.X, std::max(a.X, XMax));
        YMin = std::min(b.Y, std::min(a.Y, YMin));
        YMax = std::max(b.Y, std::max(a.Y, YMax));
    }

    bool Inside(Point p) const { return p.X >= XMin && p.X <= XMax && p.Y >= YMin && p.Y <= YMax; }

    bool Overlap(const AABB& other) const
    {
        return !(XMax < other.XMin || XMin > other.XMax || YMax < other.YMin || YMin > other.YMax);
    };

    bool Intersects(const LineSegment& lineSegment) const;

    Point TopLeft() const { return Point{XMin, YMax}; };
    Point TopRight() const { return Point{XMax, YMax}; };
    Point BottomLeft() const { return Point{XMin, YMin}; };
    Point BottomRight() const { return Point{XMax, YMin}; };
};
