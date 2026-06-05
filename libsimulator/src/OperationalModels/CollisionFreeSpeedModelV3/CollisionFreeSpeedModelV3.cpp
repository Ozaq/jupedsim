// SPDX-License-Identifier: LGPL-3.0-or-later
#include "CollisionFreeSpeedModelV3.hpp"

#include "CollisionFreeSpeedModelV3Data.hpp"
#include "CollisionFreeSpeedModelV3Update.hpp"
#include "CollisionGeometry.hpp"
#include "GenericAgent.hpp"
#include "GeometricFunctions.hpp"
#include "LineSegment.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"
#include "SimulationError.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

namespace
{
constexpr double Eps = 1e-6; // Numeric lower bound to avoid division by zero in range terms.
constexpr double SideEps = 0.05; // Smooths left/right sign near centerline to reduce heading flips.
constexpr double SpacingBlendWeight =
    0.15; // Blends move-direction spacing with goal-direction spacing.
constexpr double TauTheta = 0.3; // Heading relaxation timescale [s] for temporal smoothing.
constexpr double MinReverseSpeed =
    -0.01; // Deterministic tiny reverse floor [m/s] to release local blockages.

double NeighborInfluence(
    const std::vector<GenericAgent>& neighborhood,
    const Point& pos,
    const Point& reference_direction,
    const CollisionFreeSpeedModelV3Data& model)
{
    const auto range_x = std::max(Eps, model.RangeNeighborRepulsion * model.RangeXScale);
    const auto range_y = std::max(Eps, model.RangeNeighborRepulsion * model.RangeYScale);
    const auto theta_max =
        std::clamp(model.StrengthNeighborRepulsion, 0.0, model.ThetaMaxUpperBound);

    double best_influence = 0.0;
    double best_weight = 0.0;
    for(const auto& neighbor : neighborhood) {
        const auto relative = neighbor.Pos - pos;
        const auto x = reference_direction.ScalarProduct(relative);
        if(x <= 0.0) {
            continue;
        }

        const auto signed_lateral = reference_direction.CrossProduct(relative);
        const auto y = std::abs(signed_lateral);
        const auto longitudinal_weight = std::exp(-x / range_x);
        const auto lateral_weight = std::exp(-y / range_y);
        const auto weight = longitudinal_weight * lateral_weight;
        if(weight > best_weight) {
            best_weight = weight;
            best_influence = -weight * (signed_lateral / (std::abs(signed_lateral) + SideEps));
        }
    }

    return theta_max * std::tanh(best_influence);
}
} // namespace

OperationalModelType CollisionFreeSpeedModelV3::Type() const
{
    return OperationalModelType::COLLISION_FREE_SPEED_V3;
}

OperationalModelUpdate CollisionFreeSpeedModelV3::ComputeNewPosition(
    double dT,
    const GenericAgent& ped,
    const CollisionGeometry& geometry,
    const NeighborhoodSearchType& neighborhoodSearch) const
{
    auto neighborhood = neighborhoodSearch.GetNeighboringAgents(ped.Pos, cutOffRadius);
    const auto& boundary = geometry.LineSegmentsInApproxDistanceTo(ped.Pos);

    std::erase_if(neighborhood, [&ped, &boundary](const auto& neighbor) {
        if(ped.AgentID == neighbor.AgentID) {
            return true;
        }
        const auto agent_to_neighbor = LineSegment(ped.Pos, neighbor.Pos);
        return std::any_of(
            boundary.cbegin(), boundary.cend(), [&agent_to_neighbor](const auto& segment) {
                return intersects(agent_to_neighbor, segment);
            });
    });

    const auto boundaryRepulsionForce = std::accumulate(
        boundary.cbegin(),
        boundary.cend(),
        Point(0, 0),
        [this, &ped](const auto& acc, const auto& element) {
            return acc + boundaryRepulsion(ped, element);
        });

    const auto& model = std::get<CollisionFreeSpeedModelV3Data>(ped.Model);
    const auto desired_direction = (ped.Destination - ped.Pos).Normalized();
    auto reference_direction = (desired_direction + boundaryRepulsionForce).Normalized();
    if(reference_direction == Point{}) {
        reference_direction = ped.Orientation;
    }

    const auto heading_target =
        NeighborInfluence(neighborhood, ped.Pos, reference_direction, model);
    const auto alpha = std::clamp(dT / TauTheta, 0.0, 1.0);
    const auto heading_angle = model.HeadingAngle + alpha * (heading_target - model.HeadingAngle);
    auto direction =
        reference_direction.Rotate(std::cos(heading_angle), std::sin(heading_angle)).Normalized();
    if(direction == Point{}) {
        direction = reference_direction;
    }

    const auto spacing_move = std::accumulate(
        std::begin(neighborhood),
        std::end(neighborhood),
        std::numeric_limits<double>::max(),
        [&ped, &direction, this](const auto& res, const auto& neighbor) {
            return std::min(res, getSpacing(ped, neighbor, direction));
        });

    const auto goal_direction =
        (desired_direction == Point{}) ? reference_direction : desired_direction;
    const auto spacing_goal = std::accumulate(
        std::begin(neighborhood),
        std::end(neighborhood),
        std::numeric_limits<double>::max(),
        [&ped, &goal_direction, this](const auto& res, const auto& neighbor) {
            return std::min(res, getSpacing(ped, neighbor, goal_direction));
        });

    const auto spacing =
        spacing_move * (1.0 - SpacingBlendWeight) + spacing_goal * SpacingBlendWeight;

    const auto optimal_speed = optimalSpeed(ped, spacing, model.TimeGap);
    const auto velocity = direction * optimal_speed;
    return CollisionFreeSpeedModelV3Update{ped.Pos + velocity * dT, direction, heading_angle};
};

void CollisionFreeSpeedModelV3::ApplyUpdate(const OperationalModelUpdate& upd, GenericAgent& agent)
    const
{
    const auto& update = std::get<CollisionFreeSpeedModelV3Update>(upd);
    agent.Pos = update.Position;
    agent.Orientation = update.Orientation;
    auto& model = std::get<CollisionFreeSpeedModelV3Data>(agent.Model);
    model.HeadingAngle = update.HeadingAngle;
}

void CollisionFreeSpeedModelV3::CheckModelConstraint(
    const GenericAgent& agent,
    const NeighborhoodSearchType& neighborhoodSearch,
    const CollisionGeometry& geometry) const
{
    const auto& model = std::get<CollisionFreeSpeedModelV3Data>(agent.Model);

    validateConstraint(model.Radius, 0.0, 2.0, "radius", true);
    validateConstraint(model.V0, 0.0, 10.0, "v0");
    validateConstraint(model.TimeGap, 0.1, 10.0, "timeGap");

    validateConstraint(
        model.StrengthNeighborRepulsion,
        0.0,
        std::numeric_limits<double>::max(),
        "strengthNeighborRepulsion");
    validateConstraint(
        model.RangeNeighborRepulsion,
        0.01,
        std::numeric_limits<double>::max(),
        "rangeNeighborRepulsion");
    validateConstraint(
        model.StrengthGeometryRepulsion,
        0.0,
        std::numeric_limits<double>::max(),
        "strengthGeometryRepulsion");
    validateConstraint(
        model.RangeGeometryRepulsion,
        0.01,
        std::numeric_limits<double>::max(),
        "rangeGeometryRepulsion");

    validateConstraint(model.RangeXScale, 0.01, std::numeric_limits<double>::max(), "rangeXScale");
    validateConstraint(model.RangeYScale, 0.01, std::numeric_limits<double>::max(), "rangeYScale");
    validateConstraint(model.ThetaMaxUpperBound, 0.0, std::acos(-1.0), "thetaMaxUpperBound");
    validateConstraint(model.AgentBuffer, 0.0, 100.0, "agentBuffer");

    const auto neighbors = neighborhoodSearch.GetNeighboringAgents(agent.Pos, 2);
    for(const auto& neighbor : neighbors) {
        if(agent.AgentID == neighbor.AgentID) {
            continue;
        }
        const auto& neighbor_model = std::get<CollisionFreeSpeedModelV3Data>(neighbor.Model);
        const auto contactDist = model.Radius + neighbor_model.Radius;
        const auto distance = (agent.Pos - neighbor.Pos).Norm();
        if(contactDist >= distance) {
            throw SimulationError(
                "Model constraint violation: Agent {} too close to agent {}: distance {}",
                agent.Pos,
                neighbor.Pos,
                distance);
        }
    }

    const auto lineSegments = geometry.LineSegmentsInDistanceTo(model.Radius, agent.Pos);
    if(std::begin(lineSegments) != std::end(lineSegments)) {
        throw SimulationError(
            "Model constraint violation: Agent {} too close to geometry boundaries, distance "
            "<= {}",
            agent.Pos,
            model.Radius);
    }
}

std::unique_ptr<OperationalModel> CollisionFreeSpeedModelV3::Clone() const
{
    return std::make_unique<CollisionFreeSpeedModelV3>(*this);
}

double CollisionFreeSpeedModelV3::optimalSpeed(
    const GenericAgent& ped,
    double spacing,
    double time_gap) const
{
    const auto& model = std::get<CollisionFreeSpeedModelV3Data>(ped.Model);
    const auto effective_spacing = spacing - model.AgentBuffer;
    return std::min(std::max(effective_spacing / time_gap, MinReverseSpeed), model.V0);
}

double CollisionFreeSpeedModelV3::getSpacing(
    const GenericAgent& ped1,
    const GenericAgent& ped2,
    const Point& direction) const
{
    const auto& model1 = std::get<CollisionFreeSpeedModelV3Data>(ped1.Model);
    const auto& model2 = std::get<CollisionFreeSpeedModelV3Data>(ped2.Model);
    const auto distp12 = ped2.Pos - ped1.Pos;
    if(direction.ScalarProduct(distp12) < 0.0) {
        return std::numeric_limits<double>::max();
    }

    const auto left = direction.Rotate90Deg();
    const auto l = model1.Radius + model2.Radius;
    const auto inCorridor = std::abs(left.ScalarProduct(distp12)) <= l;
    if(!inCorridor) {
        return std::numeric_limits<double>::max();
    }

    return distp12.Norm() - l;
}

Point CollisionFreeSpeedModelV3::boundaryRepulsion(
    const GenericAgent& ped,
    const LineSegment& boundary_segment) const
{
    const auto pt = boundary_segment.ShortestPoint(ped.Pos);
    const auto dist_vec = pt - ped.Pos;
    const auto [dist, e_iw] = dist_vec.NormAndNormalized();
    const auto& model = std::get<CollisionFreeSpeedModelV3Data>(ped.Model);
    const auto l = model.Radius;
    const auto R_iw =
        -model.StrengthGeometryRepulsion * std::exp((l - dist) / model.RangeGeometryRepulsion);
    return e_iw * R_iw;
}
