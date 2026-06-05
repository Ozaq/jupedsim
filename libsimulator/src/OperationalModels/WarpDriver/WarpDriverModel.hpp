// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CollisionGeometry.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"

#include <cstdint>
#include <memory>
#include <random>
#include <vector>

struct GenericAgent;

class WarpDriverModel : public OperationalModel
{
public:
    using NeighborhoodSearchType = NeighborhoodSearch<GenericAgent>;

    /// 3-component space-time point/vector used internally
    struct SpaceTimePoint {
        double X{};
        double Y{};
        double T{};
    };

private:
    /// Precomputed 2D collision probability field I(x,y) and its gradient.
    /// Constant along time axis; time is a validity window [0,1] normalized.
    struct IntrinsicField {
        std::vector<double> Values;
        std::vector<Point> Gradients; // (dI/dx, dI/dy)
        double XMin{-3.0};
        double XMax{3.0};
        double YMin{-3.0};
        double YMax{3.0};
        double Dx{0.1};
        double Dy{0.1};
        int Nx{61};
        int Ny{61};

        void Compute(double sigma);
        /// Bilinear interpolation. Returns (0, {0,0}) for out-of-bounds.
        std::pair<double, Point> Sample(double x, double y) const;
    };

    // Model-level parameters
    double timeHorizon;
    double stepSize;
    double timeUncertainty;
    double velocityUncertaintyX;
    double velocityUncertaintyY;
    int numSamples;
    double cutOffRadius;

    IntrinsicField intrinsicField;
    mutable std::mt19937 rng;

public:
    WarpDriverModel(
        double timeHorizon,
        double stepSize,
        double sigma,
        double timeUncertainty,
        double velocityUncertaintyX,
        double velocityUncertaintyY,
        int numSamples,
        uint64_t rngSeed = 42);

    ~WarpDriverModel() override = default;

    OperationalModelType Type() const override;

    OperationalModelUpdate ComputeNewPosition(
        double dT,
        const GenericAgent& ped,
        const CollisionGeometry& geometry,
        const NeighborhoodSearchType& neighborhoodSearch) const override;

    void ApplyUpdate(const OperationalModelUpdate& update, GenericAgent& agent) const override;

    void CheckModelConstraint(
        const GenericAgent& agent,
        const NeighborhoodSearchType& neighborhoodSearch,
        const CollisionGeometry& geometry) const override;

    std::unique_ptr<OperationalModel> Clone() const override;
};
