// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "generic_agent.hpp"
#include "stage.hpp"
#include "stage_manager.hpp"

#include <boost/container/deque.hpp>

#include <string>
#include <unordered_map>

template <typename Agent>
class AgentRemovalSystem
{
public:
    AgentRemovalSystem() = default;
    ~AgentRemovalSystem() = default;
    AgentRemovalSystem(const AgentRemovalSystem& other) = delete;
    AgentRemovalSystem& operator=(const AgentRemovalSystem& other) = delete;
    AgentRemovalSystem(AgentRemovalSystem&& other) = delete;
    AgentRemovalSystem& operator=(AgentRemovalSystem&& other) = delete;

    /// Removes the agents that leave the simulation, see GenericAgent::leaving, and with each
    /// of them what else knows of it: its stage in @p stage_manager no longer counts it, and
    /// its name in @p agent_ids_by_name is free again. Nothing in here throws.
    void
    run(AgentContainer<Agent>& agents,
        StageManager& stage_manager,
        std::unordered_map<std::string, GenericAgent::ID>& agent_ids_by_name) const;
};

template <typename Agent>
void AgentRemovalSystem<Agent>::run(
    AgentContainer<Agent>& agents,
    StageManager& stage_manager,
    std::unordered_map<std::string, GenericAgent::ID>& agent_ids_by_name) const
{
    // The predicate is called once for every agent, where it stands and before anything is
    // moved over it: the place to forget an agent that goes.
    boost::container::erase_if(agents, [&](const GenericAgent& agent) {
        if(!agent.leaving) {
            return false;
        }
        if(agent.stage_id != BaseStage::ID::invalid) {
            stage_manager.handle_remove_agent(agent.stage_id);
        }
        if(!agent.name.empty()) {
            agent_ids_by_name.erase(agent.name);
        }
        return true;
    });
}
