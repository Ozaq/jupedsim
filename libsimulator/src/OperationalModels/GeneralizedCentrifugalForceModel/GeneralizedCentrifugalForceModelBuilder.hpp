// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "GeneralizedCentrifugalForceModel.hpp"

class GeneralizedCentrifugalForceModelBuilder
{
    double nuped;
    double nuwall;
    double distEffPed;
    double distEffWall;
    double intpWidthped;
    double intpWidthwall;
    double maxfped;
    double maxfwall;

public:
    GeneralizedCentrifugalForceModelBuilder(
        double nuped,
        double nuwall,
        double dist_effPed,
        double dist_effWall,
        double intp_widthped,
        double intp_widthwall,
        double maxfped,
        double maxfwall);
    GeneralizedCentrifugalForceModel Build();
};
