// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Point.hpp"

struct WarpDriverModelUpdate {
    Point Position{};
    Point Orientation{};
    double StuckTime{0.0};
    double AnchorX{0.0};
    double AnchorY{0.0};
    double DetourTime{0.0};
    int DetourSide{1};
};
