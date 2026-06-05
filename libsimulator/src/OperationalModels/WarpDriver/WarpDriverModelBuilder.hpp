// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "WarpDriverModel.hpp"

class WarpDriverModelBuilder
{
    double timeHorizon;
    double stepSize;
    double sigma;
    double timeUncertainty;
    double velocityUncertaintyX;
    double velocityUncertaintyY;

public:
    WarpDriverModelBuilder(
        double timeHorizon = 2.0,
        double stepSize = 0.5,
        double sigma = 0.3,
        double timeUncertainty = 0.5,
        double velocityUncertaintyX = 0.2,
        double velocityUncertaintyY = 0.2);
    WarpDriverModel Build();
};
