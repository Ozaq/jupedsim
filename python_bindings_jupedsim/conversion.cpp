// SPDX-License-Identifier: LGPL-3.0-or-later
#include "conversion.hpp"

#include <algorithm>
#include <iterator>

std::tuple<double, double> intoTuple(const Point& p)
{
    return std::make_tuple(p.x, p.y);
}

std::vector<std::tuple<double, double>> intoTuple(const std::vector<Point>& p)
{
    std::vector<std::tuple<double, double>> res;
    res.reserve(p.size());
    std::transform(
        std::begin(p), std::end(p), std::back_inserter(res), [](auto&& x) { return intoTuple(x); });
    return res;
}

std::vector<std::tuple<double, double>> intoTuple(const Point* beg, const Point* end)
{
    std::vector<std::tuple<double, double>> res;
    res.reserve(end - beg);
    std::transform(beg, end, std::back_inserter(res), [](auto&& x) { return intoTuple(x); });
    return res;
}

Point intoJPS_Point(const std::tuple<double, double> p)
{
    return Point{std::get<0>(p), std::get<1>(p)};
};

std::vector<Point> intoJPS_Point(const std::vector<std::tuple<double, double>>& p)
{
    std::vector<Point> res;
    res.reserve(p.size());
    std::transform(std::begin(p), std::end(p), std::back_inserter(res), [](auto&& x) {
        return intoJPS_Point(x);
    });
    return res;
}
