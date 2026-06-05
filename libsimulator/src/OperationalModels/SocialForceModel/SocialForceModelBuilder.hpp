// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "SocialForceModel.hpp"
class SocialForceModelBuilder
{
    double bodyForce;
    double friction;

public:
    SocialForceModelBuilder(double bodyForce, double friction);
    SocialForceModel Build();
};
