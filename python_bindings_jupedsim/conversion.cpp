// SPDX-License-Identifier: LGPL-3.0-or-later
#include "conversion.hpp"

std::tuple<double, double> intoTuple(const Point& p)
{
    return std::make_tuple(p.x, p.y);
}

Point intoPoint(const std::tuple<double, double>& p)
{
    return Point{std::get<0>(p), std::get<1>(p)};
}
