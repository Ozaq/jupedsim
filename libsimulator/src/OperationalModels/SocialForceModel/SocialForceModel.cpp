// SPDX-License-Identifier: LGPL-3.0-or-later
#include "SocialForceModel.hpp"

#include "CollisionGeometry.hpp"
#include "GenericAgent.hpp"
#include "LineSegment.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"
#include "SimulationError.hpp"
#include "SocialForceModelData.hpp"

#include <cmath>
#include <iterator>
#include <memory>
#include <numeric>
#include <string>

SocialForceModel::SocialForceModel(double bodyForce_, double friction_)
    : bodyForce(bodyForce_), friction(friction_) {};

OperationalModelType SocialForceModel::Type() const
{
    return OperationalModelType::SOCIAL_FORCE;
}

std::unique_ptr<OperationalModel> SocialForceModel::Clone() const
{
    return std::make_unique<SocialForceModel>(*this);
}

OperationalModelUpdate SocialForceModel::ComputeNewPosition(
    double dT,
    const GenericAgent& ped,
    const CollisionGeometry& geometry,
    const NeighborhoodSearchType& neighborhoodSearch) const
{
    const auto& model = std::get<SocialForceModelData>(ped.Model);
    SocialForceModelUpdate update{};
    auto forces = drivingForce(ped);

    const auto neighborhood = neighborhoodSearch.GetNeighboringAgents(ped.Pos, this->cutOffRadius);
    Point F_rep;
    for(const auto& neighbor : neighborhood) {
        if(neighbor.AgentID == ped.AgentID) {
            continue;
        }
        F_rep += agentForce(ped, neighbor);
    }
    forces += F_rep / model.Mass;
    const auto& walls = geometry.LineSegmentsInApproxDistanceTo(ped.Pos);

    const auto obstacle_f = std::accumulate(
        walls.cbegin(),
        walls.cend(),
        Point(0, 0),
        [this, &ped](const auto& acc, const auto& element) {
            return acc + obstacleForce(ped, element);
        });
    forces += obstacle_f / model.Mass;

    update.Velocity = model.Velocity + forces * dT;
    update.Position = ped.Pos + update.Velocity * dT;

    return update;
}

void SocialForceModel::ApplyUpdate(const OperationalModelUpdate& update, GenericAgent& agent) const
{
    auto& model = std::get<SocialForceModelData>(agent.Model);
    const auto& upd = std::get<SocialForceModelUpdate>(update);
    agent.Pos = upd.Position;
    model.Velocity = upd.Velocity;
    agent.Orientation = upd.Velocity.Normalized();
}

void SocialForceModel::CheckModelConstraint(
    const GenericAgent& agent,
    const NeighborhoodSearchType& neighborhoodSearch,
    const CollisionGeometry& geometry) const
{
    // none of these constraint are given by the paper but are useful to create a simulation that
    // does not break immediately
    auto throwIfNegative = [](double value, std::string name) {
        if(value < 0) {
            throw SimulationError(
                "Model constraint violation: {} {} not in allowed range, "
                "{} needs to be positive",
                name,
                value,
                name);
        }
    };

    const auto& model = std::get<SocialForceModelData>(agent.Model);

    const auto mass = model.Mass;
    throwIfNegative(mass, "mass");

    const auto desiredSpeed = model.DesiredSpeed;
    throwIfNegative(desiredSpeed, "desired speed");

    const auto reactionTime = model.ReactionTime;
    throwIfNegative(reactionTime, "reaction time");

    const auto radius = model.Radius;
    throwIfNegative(radius, "radius");

    const auto neighbors = neighborhoodSearch.GetNeighboringAgents(agent.Pos, 2);
    for(const auto& neighbor : neighbors) {
        const auto distance = (agent.Pos - neighbor.Pos).Norm();

        if(model.Radius >= distance) {
            throw SimulationError(
                "Model constraint violation: Agent {} too close to agent {}: distance {}, "
                "radius {}",
                agent.Pos,
                neighbor.Pos,
                distance,
                model.Radius);
        }
    }
    const auto maxRadius = model.Radius / 2;
    const auto lineSegments = geometry.LineSegmentsInDistanceTo(maxRadius, agent.Pos);
    if(std::begin(lineSegments) != std::end(lineSegments)) {
        throw SimulationError(
            "Model constraint violation: Agent {} too close to geometry boundaries, distance <= "
            "{}/2",
            agent.Pos,
            model.Radius);
    }
}

Point SocialForceModel::drivingForce(const GenericAgent& agent)
{
    const auto& model = std::get<SocialForceModelData>(agent.Model);
    const Point e0 = (agent.Destination - agent.Pos).Normalized();
    return (e0 * model.DesiredSpeed - model.Velocity) / model.ReactionTime;
};
double SocialForceModel::pushingForceLength(double A, double B, double r, double distance)
{
    return A * exp((r - distance) / B);
}

Point SocialForceModel::agentForce(const GenericAgent& ped1, const GenericAgent& ped2) const
{
    const auto& model1 = std::get<SocialForceModelData>(ped1.Model);
    const auto& model2 = std::get<SocialForceModelData>(ped2.Model);

    const double total_radius = model1.Radius + model2.Radius;

    return forceBetweenPoints(
        ped1.Pos,
        ped2.Pos,
        model1.AgentScale,
        model1.ForceDistance,
        total_radius,
        model2.Velocity - model1.Velocity);
};

Point SocialForceModel::obstacleForce(const GenericAgent& agent, const LineSegment& segment) const
{
    const auto& model = std::get<SocialForceModelData>(agent.Model);
    const Point pt = segment.ShortestPoint(agent.Pos);
    return forceBetweenPoints(
        agent.Pos, pt, model.ObstacleScale, model.ForceDistance, model.Radius, model.Velocity);
}

Point SocialForceModel::forceBetweenPoints(
    const Point pt1,
    const Point pt2,
    const double A,
    const double B,
    const double radius,
    const Point velocity) const
{
    // todo reduce range of force to 180 degrees
    const double dist = (pt1 - pt2).Norm();
    double pushing_force_length = pushingForceLength(A, B, radius, dist);
    double friction_force_length = 0;
    const Point n_ij = (pt1 - pt2).Normalized();
    const Point tangent = n_ij.Rotate90Deg();
    if(dist < radius) {
        pushing_force_length += this->bodyForce * (radius - dist);
        friction_force_length =
            this->friction * (radius - dist) * (velocity.ScalarProduct(tangent));
    }
    return n_ij * pushing_force_length + tangent * friction_force_length;
}
