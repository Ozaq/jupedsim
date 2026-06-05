// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Simulation.hpp"

#include "CollisionGeometry.hpp"
#include "GeneralizedCentrifugalForceModelData.hpp"
#include "GenericAgent.hpp"
#include "GeometrySwitchError.hpp"
#include "IteratorPair.hpp"
#include "Journey.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"
#include "Polygon.hpp"
#include "RoutingEngine.hpp"
#include "SimulationClock.hpp"
#include "SimulationError.hpp"
#include "Stage.hpp"
#include "StageDescription.hpp"
#include "Tracing.hpp"
#include "Visitor.hpp"

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

Simulation::Simulation(
    std::unique_ptr<OperationalModel>&& operationalModel,
    std::unique_ptr<CollisionGeometry>&& geometry,
    double dT)
    : clock(dT), operationalDecisionSystem(std::move(operationalModel))
{
    const auto p = geometry->Polygon();
    const auto& [tup, res] = geometries.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(geometry->GetID()),
        std::forward_as_tuple(std::move(geometry), std::make_unique<RoutingEngine>(p)));
    if(!res) {
        throw SimulationError("Internal error");
    }
    this->geometry = std::get<0>(tup->second).get();
    routingEngine = std::get<1>(tup->second).get();
}
const SimulationClock& Simulation::Clock() const
{
    return clock;
}

void Simulation::SetTracing(bool status)
{
    if(status) {
        Profiler::Instance().Enable();
    } else {
        Profiler::Instance().Disable();
    }
};

void Simulation::Iterate()
{
    JPS_SCOPED_TIMER_AND_TRACE(timer, "Total Iteration", General);

    {
        JPS_SCOPED_TIMER_AND_TRACE(timer, "Agent Removal System", Detailed);
        agentRemovalSystem.Run(agents, removedAgentsInLastIteration, stageManager);
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(timer, "Neighborhood Search", Detailed);
        neighborhoodSearch.Update(agents);
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(timer, "Stage System", Detailed);
        stageSystem.Run(stageManager, neighborhoodSearch, *geometry);
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(timer, "Strategical Decision System", General);
        stategicalDecisionSystem.Run(journeys, agents, stageManager);
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(timer, "Tactical Decision System", General);
        tacticalDecisionSystem.Run(*routingEngine, agents);
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(timer, "Operational Decision System", General);
        operationalDecisionSystem.Run(
            clock.DT(), clock.ElapsedTime(), neighborhoodSearch, *geometry, agents);
    }
    clock.Advance();
}

Journey::ID Simulation::AddJourney(const std::map<BaseStage::ID, TransitionDescription>& stages)
{
    JPS_SCOPED_TIMER_AND_TRACE(timer, "Add Journey", Detailed);
    std::map<BaseStage::ID, JourneyNode> nodes;
    bool containsDirectSteering =
        std::find_if(std::begin(stages), std::end(stages), [this](auto const& pair) {
            return std::holds_alternative<DirectSteeringProxy>(Stage(pair.first));
        }) != std::end(stages);

    if(containsDirectSteering && stages.size() > 1) {
        throw SimulationError(
            "Journeys containing a DirectSteeringStage, may only contain this stage.");
    }

    std::transform(
        std::begin(stages),
        std::end(stages),
        std::inserter(nodes, std::end(nodes)),
        [this](auto const& pair) -> std::pair<BaseStage::ID, JourneyNode> {
            const auto& [id, desc] = pair;
            auto stage = stageManager.Stage(id);
            return {
                id,
                JourneyNode{
                    stage,
                    std::visit(
                        overloaded{
                            [stage](
                                const NonTransitionDescription&) -> std::unique_ptr<Transition> {
                                return std::make_unique<FixedTransition>(stage);
                            },
                            [this](const FixedTransitionDescription& d)
                                -> std::unique_ptr<Transition> {
                                return std::make_unique<FixedTransition>(
                                    stageManager.Stage(d.NextId()));
                            },
                            [this](const RoundRobinTransitionDescription& d)
                                -> std::unique_ptr<Transition> {
                                std::vector<std::tuple<BaseStage*, uint64_t>> weightedStages{};
                                weightedStages.reserve(d.WeightedStages().size());

                                std::transform(
                                    std::begin(d.WeightedStages()),
                                    std::end(d.WeightedStages()),
                                    std::back_inserter(weightedStages),
                                    [this](auto const& pair) -> std::tuple<BaseStage*, uint64_t> {
                                        const auto& [id, weight] = pair;
                                        return {stageManager.Stage(id), weight};
                                    });

                                return std::make_unique<RoundRobinTransition>(weightedStages);
                            },
                            [this](const LeastTargetedTransitionDescription& d)
                                -> std::unique_ptr<Transition> {
                                std::vector<BaseStage*> candidates{};
                                candidates.reserve(d.TargetCandidates().size());

                                std::transform(
                                    std::begin(d.TargetCandidates()),
                                    std::end(d.TargetCandidates()),
                                    std::back_inserter(candidates),
                                    [this](auto const& id) -> BaseStage* {
                                        return stageManager.Stage(id);
                                    });

                                return std::make_unique<LeastTargetedTransition>(candidates);
                            }},
                        desc)}};
        });

    auto journey = std::make_unique<Journey>(std::move(nodes));
    const auto id = journey->GetID();
    journeys.emplace(id, std::move(journey));
    return id;
}

BaseStage::ID Simulation::AddStage(const StageDescription stageDescription)
{
    JPS_SCOPED_TIMER_AND_TRACE(timer, "Add Stage", Detailed);
    std::visit(
        overloaded{
            [this](const WaypointDescription& d) -> void {
                if(!this->geometry->InsideGeometry(d.Position)) {
                    throw SimulationError("WayPoint {} not inside walkable area", d.Position);
                }
            },
            [this](const ExitDescription& d) -> void {
                if(!this->geometry->InsideGeometry(d.Area.Centroid())) {
                    throw SimulationError("Exit {} not inside walkable area", d.Area.Centroid());
                }
            },
            [this](const NotifiableWaitingSetDescription& d) -> void {
                for(const auto& point : d.Slots) {
                    if(!this->geometry->InsideGeometry(point)) {
                        throw SimulationError(
                            "NotifiableWaitingSet point {} not inside walkable area", point);
                    }
                }
            },
            [this](const NotifiableQueueDescription& d) -> void {
                for(const auto& point : d.Slots) {
                    if(!this->geometry->InsideGeometry(point)) {
                        throw SimulationError(
                            "NotifiableQueue point {} not inside walkable area", point);
                    }
                }
            },
            [](const DirectSteeringDescription&) -> void {

            }},
        stageDescription);

    return stageManager.AddStage(stageDescription, removedAgentsInLastIteration);
}

GenericAgent::ID Simulation::AddAgent(GenericAgent agent)
{
    JPS_SCOPED_TIMER_AND_TRACE(timer, "Add Agent", Detailed);
    if(!geometry->InsideGeometry(agent.Pos)) {
        throw SimulationError("Agent {} not inside walkable area", agent.Pos);
    }
    if(journeys.count(agent.JourneyID) == 0) {
        throw SimulationError("Unknown journey id: {}", agent.JourneyID);
    }

    if(!journeys.at(agent.JourneyID)->ContainsStage(agent.StageID)) {
        throw SimulationError("Unknown stage id: {}", agent.StageID);
    }

    if(std::holds_alternative<GeneralizedCentrifugalForceModelData>(agent.Model))
        if(agent.Orientation.IsZeroLength()) {
            throw SimulationError(
                "Orientation is invalid: {}. Length should be 1.", agent.Orientation);
        }

    agent.Orientation = agent.Orientation.Normalized();
    operationalDecisionSystem.ValidateAgent(agent, neighborhoodSearch, *geometry);

    stageManager.HandleNewAgent(agent.StageID);
    agents.emplace_back(std::move(agent));
    neighborhoodSearch.AddAgent(agents.back());

    auto v = IteratorPair(std::prev(std::end(agents)), std::end(agents));
    stategicalDecisionSystem.Run(journeys, v, stageManager);
    tacticalDecisionSystem.Run(*routingEngine, v);
    return agents.back().AgentID.GetID();
}

void Simulation::MarkAgentForRemoval(GenericAgent::ID id)
{
    JPS_TRACE_FUNC;
    const auto iter = std::find_if(
        std::begin(agents), std::end(agents), [id](auto& agent) { return agent.AgentID == id; });
    if(iter == std::end(agents)) {
        throw SimulationError("Unknown agent id {}", id);
    }

    removedAgentsInLastIteration.push_back(id);
}

const GenericAgent& Simulation::Agent(GenericAgent::ID id) const
{
    JPS_TRACE_FUNC;
    const auto iter =
        std::find_if(agents.begin(), agents.end(), [id](auto& ped) { return id == ped.AgentID; });
    if(iter == agents.end()) {
        throw SimulationError("Trying to access unknown Agent {}", id);
    }
    return *iter;
}

GenericAgent& Simulation::Agent(GenericAgent::ID id)
{
    JPS_TRACE_FUNC;
    const auto iter =
        std::find_if(agents.begin(), agents.end(), [id](auto& ped) { return id == ped.AgentID; });
    if(iter == agents.end()) {
        throw SimulationError("Trying to access unknown Agent {}", id);
    }
    return *iter;
}

const std::vector<GenericAgent::ID>& Simulation::RemovedAgents() const
{
    return removedAgentsInLastIteration;
}

double Simulation::ElapsedTime() const
{
    return clock.ElapsedTime();
}

double Simulation::DT() const
{
    return clock.DT();
}

uint64_t Simulation::Iteration() const
{
    return clock.Iteration();
}

size_t Simulation::AgentCount() const
{
    return agents.size();
}

std::vector<GenericAgent>& Simulation::Agents()
{
    return agents;
};

void Simulation::SwitchAgentJourney(
    GenericAgent::ID agent_id,
    Journey::ID journey_id,
    BaseStage::ID stage_id)
{
    JPS_TRACE_FUNC;
    const auto find_iter = journeys.find(journey_id);
    if(find_iter == std::end(journeys)) {
        throw SimulationError("Unknown Journey id {}", journey_id);
    }
    auto& journey = find_iter->second;
    if(!journey->ContainsStage(stage_id)) {
        throw SimulationError("Stage {} not part of Journey {}", stage_id, journey_id);
    }
    auto& agent = Agent(agent_id);
    agent.JourneyID = journey_id;
    stageManager.MigrateAgent(agent.StageID, stage_id);
    agent.StageID = stage_id;
}

std::vector<GenericAgent::ID> Simulation::AgentsInRange(Point p, double distance)
{
    JPS_SCOPED_TIMER_AND_TRACE(timer, "Agents in Range", Debug);
    const auto neighbors = neighborhoodSearch.GetNeighboringAgents(p, distance);

    std::vector<GenericAgent::ID> neighborIds{};
    neighborIds.reserve(neighbors.size());
    std::transform(
        std::begin(neighbors),
        std::end(neighbors),
        std::back_inserter(neighborIds),
        [](const auto& agent) { return agent.AgentID; });
    return neighborIds;
}

std::vector<GenericAgent::ID> Simulation::AgentsInPolygon(const std::vector<Point>& polygon)
{
    JPS_SCOPED_TIMER_AND_TRACE(timer, "Agents in Polygon", Debug);
    const Polygon poly{polygon};
    if(!poly.IsConvex()) {
        throw SimulationError("Polygon needs to be simple and convex");
    }
    const auto [p, dist] = poly.ContainingCircle();

    const auto candidates = neighborhoodSearch.GetNeighboringAgents(p, dist);
    std::vector<GenericAgent::ID> result{};
    result.reserve(candidates.size());
    std::for_each(
        std::begin(candidates), std::end(candidates), [&result, &poly](const auto& agent) {
            if(poly.IsInside(agent.Pos)) {
                result.push_back(agent.AgentID);
            }
        });
    return result;
}

OperationalModelType Simulation::ModelType() const
{
    return operationalDecisionSystem.ModelType();
}

StageProxy Simulation::Stage(BaseStage::ID stageId)
{
    return stageManager.Stage(stageId)->Proxy(this);
}
CollisionGeometry Simulation::Geo() const
{
    return *geometry;
}

void Simulation::SwitchGeometry(std::unique_ptr<CollisionGeometry>&& geometry)
{
    JPS_TRACE_FUNC;
    validateGeometry(geometry);
    if(const auto& iter = geometries.find(geometry->GetID()); iter != std::end(geometries)) {
        this->geometry = std::get<0>(iter->second).get();
        routingEngine = std::get<1>(iter->second).get();
    } else {
        const auto p = geometry->Polygon();
        const auto& [tup, res] = geometries.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(geometry->GetID()),
            std::forward_as_tuple(std::move(geometry), std::make_unique<RoutingEngine>(p)));
        if(!res) {
            throw SimulationError("Internal error");
        }
        this->geometry = std::get<0>(tup->second).get();
        routingEngine = std::get<1>(tup->second).get();
    }
}

void Simulation::validateGeometry(const std::unique_ptr<CollisionGeometry>& geometry) const
{
    JPS_TRACE_FUNC;
    std::vector<GenericAgent::ID> faultyAgents;
    for(const auto& agent : agents) {
        if(const auto find_iter = std::find(
               std::begin(removedAgentsInLastIteration),
               std::end(removedAgentsInLastIteration),
               agent.AgentID);
           find_iter != std::end(removedAgentsInLastIteration)) {
            continue;
        }

        if(!geometry->InsideGeometry(agent.Pos)) {
            faultyAgents.push_back(agent.AgentID);
        }
    }

    std::vector<BaseStage::ID> faultyStages;
    for(const auto& [_, journey] : journeys) {
        for(const auto& [stageId, node] : journey->Stages()) {

            if(auto exit = dynamic_cast<Exit*>(node.Stage); exit != nullptr) {
                if(!geometry->InsideGeometry(exit->Position().Centroid())) {
                    faultyStages.push_back(stageId);
                }
            } else if(auto waypoint = dynamic_cast<Waypoint*>(node.Stage); waypoint != nullptr) {
                if(!geometry->InsideGeometry(waypoint->Position())) {
                    faultyStages.push_back(stageId);
                }
            } else if(auto queue = dynamic_cast<NotifiableQueue*>(node.Stage); queue != nullptr) {
                for(const auto& point : queue->Slots()) {
                    if(!geometry->InsideGeometry(point)) {
                        faultyStages.push_back(stageId);
                    }
                }
            } else if(auto waitingset = dynamic_cast<NotifiableWaitingSet*>(node.Stage);
                      waitingset != nullptr) {
                for(const auto& point : waitingset->Slots()) {
                    if(!geometry->InsideGeometry(point)) {
                        faultyStages.push_back(stageId);
                    }
                }
            }
        }
    }

    if(!faultyAgents.empty() || !faultyStages.empty()) {
        std::string message = "Could not switch the geometry.\n";

        if(!faultyAgents.empty()) {
            message += fmt::format(
                "The following agents would be outside of the new geometry: {}\n",
                fmt::join(faultyAgents, ", "));
        }
        if(!faultyStages.empty()) {
            message += fmt::format(
                "The following stages would be outside of the new geometry: {}",
                fmt::join(faultyStages, ", "));
        }

        throw GeometrySwitchError(message.c_str(), faultyAgents, faultyStages);
    }
}

void Simulation::PushTimer(const std::string_view name, size_t probe_log_level)
{
    timer.PushTimerProbe(name, probe_log_level);
}

void Simulation::PopTimer(const std::string_view name)
{
    timer.PopTimerProbe(name);
}

TimerEntry::duration_type Simulation::GetTimerDuration(const std::string_view name) const
{
    return timer.GetDuration(name);
}

std::map<std::string, TimerEntry::duration_type> Simulation::GetTimerDurations() const
{
    return timer.GetDurations();
}
