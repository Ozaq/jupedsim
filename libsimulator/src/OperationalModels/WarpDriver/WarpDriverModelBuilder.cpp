// SPDX-License-Identifier: LGPL-3.0-or-later
#include "WarpDriverModelBuilder.hpp"

#include "SimulationError.hpp"

namespace
{
// Internal sampling and RNG defaults. Not exposed through the public
// API: the trajectory sample count trades cost for accuracy and the
// seed is fixed for reproducibility of the symmetry-breaking
// perturbation.
constexpr int kDefaultNumSamples = 20;
constexpr uint64_t kDefaultRngSeed = 42;
} // namespace

WarpDriverModelBuilder::WarpDriverModelBuilder(
    double timeHorizon,
    double stepSize,
    double sigma,
    double timeUncertainty,
    double velocityUncertaintyX,
    double velocityUncertaintyY)
    : timeHorizon(timeHorizon)
    , stepSize(stepSize)
    , sigma(sigma)
    , timeUncertainty(timeUncertainty)
    , velocityUncertaintyX(velocityUncertaintyX)
    , velocityUncertaintyY(velocityUncertaintyY)
{
}

WarpDriverModel WarpDriverModelBuilder::Build()
{
    if(timeHorizon <= 0.0) {
        throw SimulationError(
            "WarpDriverModelBuilder: timeHorizon must be > 0, got {}", timeHorizon);
    }
    if(stepSize <= 0.0) {
        throw SimulationError("WarpDriverModelBuilder: stepSize must be > 0, got {}", stepSize);
    }
    if(sigma <= 0.0) {
        throw SimulationError("WarpDriverModelBuilder: sigma must be > 0, got {}", sigma);
    }
    if(timeUncertainty < 0.0) {
        throw SimulationError(
            "WarpDriverModelBuilder: timeUncertainty must be >= 0, got {}", timeUncertainty);
    }
    if(velocityUncertaintyX < 0.0) {
        throw SimulationError(
            "WarpDriverModelBuilder: velocityUncertaintyX must be >= 0, got {}",
            velocityUncertaintyX);
    }
    if(velocityUncertaintyY < 0.0) {
        throw SimulationError(
            "WarpDriverModelBuilder: velocityUncertaintyY must be >= 0, got {}",
            velocityUncertaintyY);
    }

    return WarpDriverModel(
        timeHorizon,
        stepSize,
        sigma,
        timeUncertainty,
        velocityUncertaintyX,
        velocityUncertaintyY,
        kDefaultNumSamples,
        kDefaultRngSeed);
}
