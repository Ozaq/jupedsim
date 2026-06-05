// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CollisionFreeSpeedModel.hpp"
class CollisionFreeSpeedModelBuilder
{
    double aPed;
    double dPed;
    double aWall;
    double dWall;

public:
    CollisionFreeSpeedModelBuilder(double aPed, double DPed, double aWall, double DWall);
    CollisionFreeSpeedModel Build();
};
