// SPDX-License-Identifier: LGPL-3.0-or-later
#include "CollisionFreeSpeedModelBuilder.hpp"

#include "CollisionFreeSpeedModel.hpp"

CollisionFreeSpeedModelBuilder::CollisionFreeSpeedModelBuilder(
    double aPed,
    double DPed,
    double aWall,
    double DWall)
    : aPed(aPed), dPed(DPed), aWall(aWall), dWall(DWall)
{
}

CollisionFreeSpeedModel CollisionFreeSpeedModelBuilder::Build()
{
    return CollisionFreeSpeedModel(aPed, dPed, aWall, dWall);
}
