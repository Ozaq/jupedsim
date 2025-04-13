// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <Point.hpp>

#include <tuple>
#include <vector>

std::tuple<double, double> intoTuple(const Point& p);

std::vector<std::tuple<double, double>> intoTuple(const std::vector<Point>& p);

std::vector<std::tuple<double, double>> intoTuple(const Point* beg, const Point* end);

Point intoJPS_Point(const std::tuple<double, double> p);

std::vector<Point> intoJPS_Point(const std::vector<std::tuple<double, double>>& p);
