// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Point.hpp"
#include "Polygon.hpp"

#include <variant>
#include <vector>

struct DirectSteeringDescription {
};

struct WaypointDescription {
    Point Position;
    double Distance;
};

struct ExitDescription {
    Polygon Area;
};

struct NotifiableWaitingSetDescription {
    std::vector<Point> Slots;
};

struct NotifiableQueueDescription {
    std::vector<Point> Slots;
};

using StageDescription = std::variant<
    DirectSteeringDescription,
    WaypointDescription,
    ExitDescription,
    NotifiableWaitingSetDescription,
    NotifiableQueueDescription>;
