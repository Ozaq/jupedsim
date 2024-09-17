#pragma once

#include <cassert>
#include <type_traits>
#include <vector>

namespace distance
{

template <typename T>
concept Real = std::is_floating_point_v<T>;

template <Real Real>
struct Point {
    Real x;
    Real y;
};

template <Real Real>
struct LineSegment {
    Point<Real> p1;
    Point<Real> p2;
};

template <Real Real>
using Polygon = std::vector<Point<Real>>;

template <typename Real>
struct Arc {
    Point<Real> center;
    Real radius;
    Real startAngle;
    Real endAngle;
};

/// Axis Aligned Bounding Box
template <Real Real>
class AABB
{
    Point<Real> lower_left;
    Point<Real> upper_right;

public:
    AABB(Point<Real> _lower_left, Point<Real> _upper_right)
        : lower_left(_lower_left), upper_right(_upper_right)
    {
        assert(lower_left.x <= upper_right.x);
        assert(lower_left.y <= upper_right.y);
    }

    Real Width() const { return upper_right.x - lower_left.x; }

    Real Height() const { return upper_right.y - lower_left.y; }

    Point<Real> LowerLeft() const { return {lower_left.x, lower_left.y}; }
};

template <Real Real>
struct Circle {
    Point<Real> center;
    Real radius;
};

// TODO(kkratz): move into grid?
struct GridIndex {
    uint64_t x{};
    uint64_t y{};
};

} // namespace distance
