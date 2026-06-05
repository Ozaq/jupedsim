// SPDX-License-Identifier: LGPL-3.0-or-later
#include "GeneralizedCentrifugalForceModelBuilder.hpp"

#include "GeneralizedCentrifugalForceModel.hpp"

GeneralizedCentrifugalForceModelBuilder::GeneralizedCentrifugalForceModelBuilder(
    double nuped,
    double nuwall,
    double dist_effPed,
    double dist_effWall,
    double intp_widthped,
    double intp_widthwall,
    double maxfped,
    double maxfwall)
    : nuped(nuped)
    , nuwall(nuwall)
    , distEffPed(dist_effPed)
    , distEffWall(dist_effWall)
    , intpWidthped(intp_widthped)
    , intpWidthwall(intp_widthwall)
    , maxfped(maxfped)
    , maxfwall(maxfwall)
{
}

GeneralizedCentrifugalForceModel GeneralizedCentrifugalForceModelBuilder::Build()
{
    return GeneralizedCentrifugalForceModel(
        nuped,
        nuwall,
        distEffPed,
        distEffWall,
        intpWidthped,
        intpWidthwall,
        maxfped,
        maxfwall);
}
