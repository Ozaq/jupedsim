// SPDX-License-Identifier: LGPL-3.0-or-later
#include "SocialForceModelBuilder.hpp"

#include "SocialForceModel.hpp"

SocialForceModelBuilder::SocialForceModelBuilder(double bodyForce, double friction)
    : bodyForce(bodyForce), friction(friction)
{
}

SocialForceModel SocialForceModelBuilder::Build()
{
    return SocialForceModel(bodyForce, friction);
}
