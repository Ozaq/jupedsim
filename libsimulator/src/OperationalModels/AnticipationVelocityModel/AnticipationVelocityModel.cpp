// SPDX-License-Identifier: LGPL-3.0-or-later
#include "AnticipationVelocityModel.hpp"

#include "AnticipationVelocityModelData.hpp"
#include "AnticipationVelocityModelUpdate.hpp"
#include "CollisionGeometry.hpp"
#include "GenericAgent.hpp"
#include "GeometricFunctions.hpp"
#include "LineSegment.hpp"
#include "Macros.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"
#include "SimulationError.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

AnticipationVelocityModel::AnticipationVelocityModel(double pushoutStrength, uint64_t rng_seed)
    : pushoutStrength(pushoutStrength), gen(rng_seed)
{
}

OperationalModelType AnticipationVelocityModel::Type() const
{
    return OperationalModelType::ANTICIPATION_VELOCITY_MODEL;
}

OperationalModelUpdate AnticipationVelocityModel::ComputeNewPosition(
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

    const auto repulsion = std::accumulate(
        std::begin(neighborhood),
        std::end(neighborhood),
        Point{},
        [&ped, this](const auto& res, const auto& neighbor) {
            return res + neighborRepulsion(ped, neighbor);
        });

    const auto desiredDirection = (ped.Destination - ped.Pos).Normalized();
    auto direction = (desiredDirection + repulsion).Normalized();
    if(direction == Point{}) {
        direction = ped.Orientation;
    }

    const auto& model = std::get<AnticipationVelocityModelData>(ped.Model);
    const double wallBufferDistance = model.WallBufferDistance;
    // Wall sliding behavior

    // update direction towards the newly calculated direction
    direction = updateDirection(ped, direction, dT);
    const auto spacing = std::accumulate(
        std::begin(neighborhood),
        std::end(neighborhood),
        std::numeric_limits<double>::max(),
        [&ped, &direction, this](const auto& res, const auto& neighbor) {
            return std::min(res, getSpacing(ped, neighbor, direction));
        });

    const auto optimal_speed = optimalSpeed(ped, spacing, model.TimeGap);
    direction = handleWallAvoidance(direction, ped.Pos, model.Radius, boundary, wallBufferDistance);

    const auto velocity = direction * optimal_speed;
    return AnticipationVelocityModelUpdate{
        .Position = ped.Pos + velocity * dT, .Velocity = velocity, .Orientation = direction};
};

void AnticipationVelocityModel::ApplyUpdate(const OperationalModelUpdate& upd, GenericAgent& agent)
    const
{
    const auto& update = std::get<AnticipationVelocityModelUpdate>(upd);
    auto& model = std::get<AnticipationVelocityModelData>(agent.Model);
    agent.Pos = update.Position;
    agent.Orientation = update.Orientation;
    model.Velocity = update.Velocity;
}

Point AnticipationVelocityModel::updateDirection(
    const GenericAgent& ped,
    const Point& calculatedDirection,
    double dt) const
{
    const auto& model = std::get<AnticipationVelocityModelData>(ped.Model);
    const Point desiredDirection = (ped.Destination - ped.Pos).Normalized();
    const Point actualDirection = ped.Orientation;
    Point updatedDirection;

    if(desiredDirection.ScalarProduct(calculatedDirection) *
           desiredDirection.ScalarProduct(actualDirection) <
       0) {
        updatedDirection = calculatedDirection;
    } else {
        // Compute the rate of change of direction (Eq. 7)
        const Point directionDerivative =
            (calculatedDirection.Normalized() - actualDirection) / model.ReactionTime;
        updatedDirection = actualDirection + directionDerivative * dt;
    }

    return updatedDirection.Normalized();
}

void AnticipationVelocityModel::CheckModelConstraint(
    const GenericAgent& agent,
    const NeighborhoodSearchType& neighborhoodSearch,
    const CollisionGeometry& geometry) const
{
    const auto& model = std::get<AnticipationVelocityModelData>(agent.Model);
    const auto r = model.Radius;
    constexpr double rMin = 0.;
    constexpr double rMax = 2.;
    validateConstraint(r, rMin, rMax, "radius", true);

    const auto strengthNeighborRepulsion = model.StrengthNeighborRepulsion;
    constexpr double snMin = 0.;
    constexpr double snMax = 20.;
    validateConstraint(strengthNeighborRepulsion, snMin, snMax, "strengthNeighborRepulsion", false);

    const auto rangeNeighborRepulsion = model.RangeNeighborRepulsion;
    constexpr double rnMin = 0.;
    constexpr double rnMax = 5.;
    validateConstraint(rangeNeighborRepulsion, rnMin, rnMax, "rangeNeighborRepulsion", true);

    const auto buff = model.WallBufferDistance;
    constexpr double buffMin = 0.;
    constexpr double buffMax = 1.;
    validateConstraint(buff, buffMin, buffMax, "wallBufferDistance", false);

    const auto v0 = model.V0;
    constexpr double v0Min = 0.;
    constexpr double v0Max = 10.;
    validateConstraint(v0, v0Min, v0Max, "v0");

    const auto timeGap = model.TimeGap;
    constexpr double timeGapMin = 0.;
    constexpr double timeGapMax = 10.;
    validateConstraint(timeGap, timeGapMin, timeGapMax, "timeGap", true);

    const auto anticipationTime = model.AnticipationTime;
    constexpr double anticipationTimeMin = 0.0;
    constexpr double anticipationTimeMax = 5.0;
    validateConstraint(
        anticipationTime, anticipationTimeMin, anticipationTimeMax, "anticipationTime");

    const auto reactionTime = model.ReactionTime;
    constexpr double reactionTimeMin = 0.0;
    constexpr double reactionTimeMax = 1.0;
    validateConstraint(reactionTime, reactionTimeMin, reactionTimeMax, "reactionTime", true);

    const auto neighbors = neighborhoodSearch.GetNeighboringAgents(agent.Pos, 2);
    for(const auto& neighbor : neighbors) {
        if(agent.AgentID == neighbor.AgentID) {
            continue;
        }
        const auto& neighbor_model = std::get<AnticipationVelocityModelData>(neighbor.Model);
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

std::unique_ptr<OperationalModel> AnticipationVelocityModel::Clone() const
{
    return std::make_unique<AnticipationVelocityModel>(*this);
}

double AnticipationVelocityModel::optimalSpeed(
    const GenericAgent& ped,
    double spacing,
    double time_gap) const
{
    const auto& model = std::get<AnticipationVelocityModelData>(ped.Model);
    constexpr double creep_speed = 0.01;

    double speed = spacing / time_gap;

    if(std::abs(speed) < creep_speed) {
        // Random shuffle: forward, backward, or stop
        const auto r = gen() % 3;
        speed = (r == 0) ? creep_speed : (r == 1) ? -creep_speed : 0.0;
    }

    return std::min(std::max(speed, -creep_speed), model.V0);
}
double AnticipationVelocityModel::getSpacing(
    const GenericAgent& ped1,
    const GenericAgent& ped2,
    const Point& direction) const
{
    const auto& model1 = std::get<AnticipationVelocityModelData>(ped1.Model);
    const auto& model2 = std::get<AnticipationVelocityModelData>(ped2.Model);
    const auto distp12 = ped2.Pos - ped1.Pos;
    const auto inFront = direction.ScalarProduct(distp12) >= 0;
    if(!inFront) {
        return std::numeric_limits<double>::max();
    }

    const auto left = direction.Rotate90Deg();
    const auto buffer = 0.02;
    const auto l = model1.Radius + model2.Radius + buffer;
    const bool inCorridor = std::abs(left.ScalarProduct(distp12)) <= l;
    if(!inCorridor) {
        return std::numeric_limits<double>::max();
    }
    return distp12.Norm() - l;
}

Point AnticipationVelocityModel::calculateInfluenceDirection(
    const Point& desiredDirection,
    const Point& predictedDirection) const
{
    // Eq. (5)
    const Point orthogonalDirection = Point(-desiredDirection.Y, desiredDirection.X).Normalized();
    const double alignment = orthogonalDirection.ScalarProduct(predictedDirection);
    Point influenceDirection = orthogonalDirection;
    if(fabs(alignment) < J_EPS) {
        // Choose a random direction (left or right)
        if(gen() % 2 == 0) {
            influenceDirection = -orthogonalDirection;
        }
    } else if(alignment > 0) {
        influenceDirection = -orthogonalDirection;
    }
    return influenceDirection;
}

Point AnticipationVelocityModel::neighborRepulsion(
    const GenericAgent& ped1,
    const GenericAgent& ped2) const
{
    const auto& model1 = std::get<AnticipationVelocityModelData>(ped1.Model);
    const auto& model2 = std::get<AnticipationVelocityModelData>(ped2.Model);

    const auto distp12 = ped2.Pos - ped1.Pos;
    const auto [distance, ep12] = distp12.NormAndNormalized();
    const double adjustedDist = distance - (model1.Radius + model2.Radius);

    // Pedestrian movement and desired directions
    const auto& e1 = ped1.Orientation;
    const auto& d1 = (ped1.Destination - ped1.Pos).Normalized();
    const auto& e2 = ped2.Orientation;

    // Check perception range (Eq. 1)
    const auto inPerceptionRange = d1.ScalarProduct(ep12) >= 0 || e1.ScalarProduct(ep12) >= 0;
    if(!inPerceptionRange)
        return Point(0, 0);

    const double S_Gap =
        (model1.Velocity - model2.Velocity).ScalarProduct(ep12) * model1.AnticipationTime;
    double R_dist = adjustedDist - S_Gap;
    R_dist = std::max(R_dist, 0.0); // Clamp to zero if negative

    // Interaction strength (Eq. 3 & 4)
    constexpr double alignmentBase = 1.0;
    constexpr double alignmentWeight = 0.5;
    const double alignmentFactor = alignmentBase + alignmentWeight * (1.0 - d1.ScalarProduct(e2));
    const double interactionStrength = model1.StrengthNeighborRepulsion * alignmentFactor *
                                       std::exp(-R_dist / model1.RangeNeighborRepulsion);
    const auto newep12 = distp12 + model2.Velocity * model2.AnticipationTime; // e_ij(t+ta)

    // Compute adjusted influence direction
    const auto influenceDirection = calculateInfluenceDirection(d1, newep12);
    return influenceDirection * interactionStrength;
}

Point AnticipationVelocityModel::handleWallAvoidance(
    const Point& direction,
    const Point& agentPosition,
    double agentRadius,
    const std::vector<LineSegment>& boundary,
    double wallBufferDistance) const
{
    const double criticalWallDistance = wallBufferDistance + agentRadius;

    Point modifiedDirection = direction;

    for(const auto& wall : boundary) {
        const auto closestPoint = wall.ShortestPoint(agentPosition);

        const auto distanceVector = agentPosition - closestPoint;
        const auto [distance, normalTowardAgent] = distanceVector.NormAndNormalized();

        if(distance > criticalWallDistance) {
            continue;
        }

        const auto dotProduct = modifiedDirection.ScalarProduct(normalTowardAgent);

        if(dotProduct < 0) {
            // Direction points into wall - need to project it out
            // Remove the component pointing into the wall
            const auto projectedDirection = modifiedDirection - normalTowardAgent * dotProduct;
            modifiedDirection = projectedDirection + normalTowardAgent * pushoutStrength;
        }
    }

    // Renormalize to maintain speed
    const auto finalDirection = modifiedDirection.Normalized();

    return finalDirection;
}
