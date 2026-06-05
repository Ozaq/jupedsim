// SPDX-License-Identifier: LGPL-3.0-or-later
#include "CollisionFreeSpeedModel.hpp"

#include "CollisionFreeSpeedModelData.hpp"
#include "CollisionFreeSpeedModelUpdate.hpp"
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
#include <cstdlib>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

CollisionFreeSpeedModel::CollisionFreeSpeedModel(
    double strengthNeighborRepulsion_,
    double rangeNeighborRepulsion_,
    double strengthGeometryRepulsion_,
    double rangeGeometryRepulsion_)
    : strengthNeighborRepulsion(strengthNeighborRepulsion_)
    , rangeNeighborRepulsion(rangeNeighborRepulsion_)
    , strengthGeometryRepulsion(strengthGeometryRepulsion_)
    , rangeGeometryRepulsion(rangeGeometryRepulsion_)
{
}

OperationalModelType CollisionFreeSpeedModel::Type() const
{
    return OperationalModelType::COLLISION_FREE_SPEED;
}

OperationalModelUpdate CollisionFreeSpeedModel::ComputeNewPosition(
    double dT,
    const GenericAgent& ped,
    const CollisionGeometry& geometry,
    const NeighborhoodSearchType& neighborhoodSearch) const
{
    auto neighborhood = neighborhoodSearch.GetNeighboringAgents(ped.Pos, cutOffRadius);
    const auto& boundary = geometry.LineSegmentsInApproxDistanceTo(ped.Pos);

    // Remove any agent from the neighborhood that is obstructed by geometry and the current
    // agent
    neighborhood.erase(
        std::remove_if(
            std::begin(neighborhood),
            std::end(neighborhood),
            [&ped, &boundary](const auto& neighbor) {
                if(ped.AgentID == neighbor.AgentID) {
                    return true;
                }
                const auto agent_to_neighbor = LineSegment(ped.Pos, neighbor.Pos);
                if(std::find_if(
                       boundary.cbegin(),
                       boundary.cend(),
                       [&agent_to_neighbor](const auto& boundary_segment) {
                           return intersects(agent_to_neighbor, boundary_segment);
                       }) != boundary.end()) {
                    return true;
                }

                return false;
            }),
        std::end(neighborhood));

    const auto neighborRepulsionForce = std::accumulate(
        std::begin(neighborhood),
        std::end(neighborhood),
        Point{},
        [&ped, this](const auto& res, const auto& neighbor) {
            return res + neighborRepulsion(ped, neighbor);
        });

    const auto boundaryRepulsionForce = std::accumulate(
        boundary.cbegin(),
        boundary.cend(),
        Point(0, 0),
        [this, &ped](const auto& acc, const auto& element) {
            return acc + boundaryRepulsion(ped, element);
        });

    const auto desired_direction = (ped.Destination - ped.Pos).Normalized();
    auto direction = (desired_direction + neighborRepulsionForce + boundaryRepulsionForce).Normalized();
    if(direction == Point{}) {
        direction = ped.Orientation;
    }
    const auto spacing = std::accumulate(
        std::begin(neighborhood),
        std::end(neighborhood),
        std::numeric_limits<double>::max(),
        [&ped, &direction, this](const auto& res, const auto& neighbor) {
            return std::min(res, getSpacing(ped, neighbor, direction));
        });

    const auto& model = std::get<CollisionFreeSpeedModelData>(ped.Model);
    const auto optimal_speed = optimalSpeed(ped, spacing, model.TimeGap);
    const auto velocity = direction * optimal_speed;
    return CollisionFreeSpeedModelUpdate{ped.Pos + velocity * dT, direction};
};

void CollisionFreeSpeedModel::ApplyUpdate(const OperationalModelUpdate& upd, GenericAgent& agent)
    const
{
    const auto& update = std::get<CollisionFreeSpeedModelUpdate>(upd);
    agent.Pos = update.Position;
    agent.Orientation = update.Orientation;
}

void CollisionFreeSpeedModel::CheckModelConstraint(
    const GenericAgent& agent,
    const NeighborhoodSearchType& neighborhoodSearch,
    const CollisionGeometry& geometry) const
{
    const auto& model = std::get<CollisionFreeSpeedModelData>(agent.Model);

    const auto r = model.Radius;
    constexpr double rMin = 0.;
    constexpr double rMax = 2.;
    validateConstraint(r, rMin, rMax, "radius", true);

    const auto v0 = model.V0;
    constexpr double v0Min = 0.;
    constexpr double v0Max = 10.;
    validateConstraint(v0, v0Min, v0Max, "v0");

    const auto timeGap = model.TimeGap;
    constexpr double timeGapMin = 0.1;
    constexpr double timeGapMax = 10.;
    validateConstraint(timeGap, timeGapMin, timeGapMax, "timeGap");

    const auto neighbors = neighborhoodSearch.GetNeighboringAgents(agent.Pos, 2);
    for(const auto& neighbor : neighbors) {
        if(agent.AgentID == neighbor.AgentID) {
            continue;
        }
        const auto& neighbor_model = std::get<CollisionFreeSpeedModelData>(neighbor.Model);
        const auto contanctdDist = r + neighbor_model.Radius;
        const auto distance = (agent.Pos - neighbor.Pos).Norm();
        if(contanctdDist >= distance) {
            throw SimulationError(
                "Model constraint violation: Agent {} too close to agent {}: distance {}",
                agent.Pos,
                neighbor.Pos,
                distance);
        }
    }

    const auto lineSegments = geometry.LineSegmentsInDistanceTo(r, agent.Pos);
    if(std::begin(lineSegments) != std::end(lineSegments)) {
        throw SimulationError(
            "Model constraint violation: Agent {} too close to geometry boundaries, distance "
            "<= {}",
            agent.Pos,
            r);
    }
}

std::unique_ptr<OperationalModel> CollisionFreeSpeedModel::Clone() const
{
    return std::make_unique<CollisionFreeSpeedModel>(*this);
}

double CollisionFreeSpeedModel::optimalSpeed(
    const GenericAgent& ped,
    double spacing,
    double time_gap) const
{
    const auto& model = std::get<CollisionFreeSpeedModelData>(ped.Model);
    return std::min(std::max(spacing / time_gap, 0.0), model.V0);
}

double CollisionFreeSpeedModel::getSpacing(
    const GenericAgent& ped1,
    const GenericAgent& ped2,
    const Point& direction) const
{
    const auto& model1 = std::get<CollisionFreeSpeedModelData>(ped1.Model);
    const auto& model2 = std::get<CollisionFreeSpeedModelData>(ped2.Model);
    const auto distp12 = ped2.Pos - ped1.Pos;
    const auto inFront = direction.ScalarProduct(distp12) >= 0;
    if(!inFront) {
        return std::numeric_limits<double>::max();
    }

    const auto left = direction.Rotate90Deg();
    const auto l = model1.Radius + model2.Radius;
    bool inCorridor = std::abs(left.ScalarProduct(distp12)) <= l;
    if(!inCorridor) {
        return std::numeric_limits<double>::max();
    }
    return distp12.Norm() - l;
}
Point CollisionFreeSpeedModel::neighborRepulsion(const GenericAgent& ped1, const GenericAgent& ped2)
    const
{
    const auto distp12 = ped2.Pos - ped1.Pos;
    const auto [distance, direction] = distp12.NormAndNormalized();
    const auto& model1 = std::get<CollisionFreeSpeedModelData>(ped1.Model);
    const auto& model2 = std::get<CollisionFreeSpeedModelData>(ped2.Model);
    const auto l = model1.Radius + model2.Radius;
    return direction * -(strengthNeighborRepulsion * exp((l - distance) / rangeNeighborRepulsion));
}

Point CollisionFreeSpeedModel::boundaryRepulsion(
    const GenericAgent& ped,
    const LineSegment& boundary_segment) const
{
    const auto pt = boundary_segment.ShortestPoint(ped.Pos);
    const auto dist_vec = pt - ped.Pos;
    const auto [dist, e_iw] = dist_vec.NormAndNormalized();
    const auto& model = std::get<CollisionFreeSpeedModelData>(ped.Model);
    const auto l = model.Radius;
    const auto R_iw = -strengthGeometryRepulsion * exp((l - dist) / rangeGeometryRepulsion);
    return e_iw * R_iw;
}
