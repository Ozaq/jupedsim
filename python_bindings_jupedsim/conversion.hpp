// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <Point.hpp>
#include <Stage.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <tuple>

std::tuple<double, double> intoTuple(const Point& p);

std::vector<std::tuple<double, double>> intoTuples(const std::vector<Point>& in);

Point intoPoint(const std::tuple<double, double>& p);

std::vector<Point> intoPoints(const std::vector<std::tuple<double, double>>& in);

namespace pybind11::detail
{

template <>
struct type_caster<BaseStage::ID> {
public:
    PYBIND11_TYPE_CASTER(BaseStage::ID, _("StageID"));

    // Python -> C++
    bool load(handle src, bool)
    {
        if(!pybind11::isinstance<pybind11::int_>(src))
            return false;
        value = BaseStage::ID(pybind11::cast<int>(src));
        return true;
    }

    // C++ -> Python
    static handle cast(const BaseStage::ID& src, return_value_policy, handle)
    {
        return pybind11::int_(src.getID()).release();
    }
};

} // namespace pybind11::detail
