// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <Point.hpp>

#include <tuple>

std::tuple<double, double> intoTuple(const Point& p);

Point intoPoint(const std::tuple<double, double>& p);
