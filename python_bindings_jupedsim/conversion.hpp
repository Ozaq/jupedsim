// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <Point.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <tuple>

std::tuple<double, double> intoTuple(const Point& p);

Point intoPoint(const std::tuple<double, double>& p);

std::vector<Point> intoPoints(const std::vector<std::tuple<double, double>>& in);
