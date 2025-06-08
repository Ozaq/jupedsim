// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <Point.hpp>

#include <pybind11/pybind11.h>

#include <tuple>

std::tuple<double, double> intoTuple(const Point& p);

Point intoPoint(const std::tuple<double, double>& p);

namespace PYBIND11_NAMESPACE
{
namespace detail
{
template <>
struct type_caster<Point> {
public:
    PYBIND11_TYPE_CASTER(Point, const_name("Point"));
    // Python -> C++
    bool load(pybind11::handle src, bool)
    {
        if(!PyTuple_Check(src.ptr()) || PyTuple_Size(src.ptr()) != 2)
            return false;

        pybind11::tuple t = pybind11::reinterpret_borrow<pybind11::tuple>(src);
        x = t[0].cast<double>();
        y = t[1].cast<double>();
        return true;
    }

    // C++ -> Python
    static pybind11::handle cast(const Point& p, return_value_policy, handle)
    {
        return pybind11::make_tuple(p.x, p.y).release();
    }

    double x, y;
};
} // namespace detail
} // namespace PYBIND11_NAMESPACE
