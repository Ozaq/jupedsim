// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Point.hpp"

#include <optional>

struct GeneralizedCentrifugalForceModelUpdate {
    std::optional<Point> Position{};
    std::optional<Point> Velocity{};
    Point E0{}; // desired direction
};
