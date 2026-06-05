// SPDX-License-Identifier: LGPL-3.0-or-later
#include "LineSegment.hpp"

#include "Macros.hpp"
#include "Point.hpp"

#include <CGAL/Distance_2/Point_2_Segment_2.h>
#include <CGAL/Simple_cartesian.h>

#include <cmath>
#include <utility>

LineSegment::LineSegment(Point _p1, Point _p2) : P1(std::move(_p1)), P2(std::move(_p2))
{
}

bool LineSegment::operator==(const LineSegment& other) const
{
    return P1 == other.P1 && P2 == other.P2;
}

bool LineSegment::operator!=(const LineSegment& other) const
{
    return !(*this == other);
}

bool LineSegment::operator<(const LineSegment& other) const
{
    if(P1 < other.P1)
        return true;
    if((P1 == other.P1) && (P2 < other.P2))
        return true;
    return false;
}

Point LineSegment::NormalVec() const
{
    const Point r = (P2 - P1);
    return Point(-r.Y, r.X).Normalized();
}

double LineSegment::NormalComp(const Point& v) const
{
    // Normierte Vectoren
    Point l = (P2 - P1).Normalized();
    const Point& n = NormalVec();

    double alpha;

    if(fabs(l.X) < J_EPS) {
        alpha = v.X / n.X;
    } else if(fabs(l.Y) < J_EPS) {
        alpha = v.Y / n.Y;
    } else {
        alpha = l.CrossProduct(v) / n.CrossProduct(l);
    }

    return fabs(alpha);
}

Point LineSegment::ShortestPoint(const Point& p) const
{
    if(P1 == P2)
        return P1;

    const Point& t = P1 - P2;
    double lambda = (p - P2).ScalarProduct(t) / t.ScalarProduct(t);
    if(lambda < 0)
        return P2;
    else if(lambda > 1)
        return P1;
    else
        return P2 + t * lambda;
}

double LineSegment::DistTo(const Point& p) const
{
    using Kernel = CGAL::Simple_cartesian<double>;
    using PointCGAL = Kernel::Point_2;
    using SegmentCGAL = Kernel::Segment_2;

    PointCGAL point(p.X, p.Y);
    SegmentCGAL segment(PointCGAL(P1.X, P1.Y), PointCGAL(P2.X, P2.Y));

    return sqrt(CGAL::squared_distance(point, segment));
}

double LineSegment::LengthSquare() const
{
    return (P1 - P2).NormSquare();
}
