// SPDX-License-Identifier: LGPL-3.0-or-later
#include "agent_view.hpp"
#include "generic_agent.hpp"
#include "geometry/geometry.hpp"
#include "geometry_fixtures.hpp"
#include "kernel.hpp"
#include "neighborhood_search.hpp"
#include "operational_decision_system.hpp"
#include "operational_model_state.hpp"
#include "operational_models/custom_model/custom_model.hpp"
#include "test_common.hpp"
#include "unique_id.hpp"

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
struct ConstantVelocity {
    Vector velocity{};
};

/// Minimal model: advances the position by `velocity * dt` each step. Enough to
/// exercise the Location change in OperationalDecisionSystem::run.
class ConstantVelocityModel : public CustomModel
{
public:
    Vector compute_next_state(
        const OperationalModelState& current,
        OperationalModelState&,
        const AgentStep& step) const override
    {
        const auto& cur = std::get<CustomModel::State>(current);
        return cur.get<ConstantVelocity>().velocity * step.dt();
    }

    void check_model_constraint(const GenericAgent&, const AgentView&) const override {}
};

GenericAgent
make_agent(const Geometry& geometry, Point start, Vector velocity, std::string name = {})
{
    return GenericAgent(
        GenericAgent::ID::invalid,
        jps::UniqueID<Route>::invalid,
        jps::UniqueID<BaseStage>::invalid,
        *geometry.get_location_near_z(start.x(), start.y(), 0.0),
        CustomModel::State{ConstantVelocity{velocity}},
        std::move(name));
}

/// A flat square [-10,10]^2 with its 3D twin over the same polygon.
std::unique_ptr<Geometry> flat_square()
{
    return test_geometries::rectangle({-10, -10}, {10, 10}).geometry;
}
} // namespace

TEST(OperationalDecisionSystemLocation, WalkingTheStepLandsWhereDeadReckoningSays)
{
    auto geo = flat_square();

    AgentContainer<GenericAgent> agents{};
    const Point start{0.0, 0.0};
    agents.emplace_back(make_agent(*geo, start, Vector{2.0, 0.0}));

    NeighborhoodSearch<GenericAgent> neighborhood_search{2.2};
    neighborhood_search.update(agents);

    OperationalDecisionSystem system{std::make_unique<ConstantVelocityModel>()};
    system.run(0.5, 0.0, neighborhood_search, *geo, agents);

    // velocity=2.0, dt=0.5 --> move x by 1.0
    const auto& agent = agents.front();
    EXPECT_NEAR(agent.location.xy().x(), start.x() + 1.0, 1e-9);
    EXPECT_NEAR(agent.location.xy().y(), start.y(), 1e-9);
    EXPECT_NEAR(agent.location.z(), 0.0, 1e-9);
    EXPECT_EQ(agent.location.region(), 0u);
}

TEST(OperationalDecisionSystemLocation, ItStaysOnTheSheetOverManySteps)
{
    auto geo = flat_square();

    AgentContainer<GenericAgent> agents{};
    const Point start{-5.0, -3.0};
    const Vector velocity{1.0, 0.5};
    const double dt = 0.1;
    const int steps = 20;
    agents.emplace_back(make_agent(*geo, start, velocity));

    NeighborhoodSearch<GenericAgent> neighborhood_search{2.2};
    OperationalDecisionSystem system{std::make_unique<ConstantVelocityModel>()};

    for(int step = 0; step < steps; ++step) {
        neighborhood_search.update(agents);
        system.run(dt, 0.0, neighborhood_search, *geo, agents);
        const auto& agent = agents.front();
        EXPECT_NEAR(agent.location.z(), 0.0, 1e-9);
        EXPECT_EQ(agent.location.region(), 0u);
    }
    // After `steps` steps the agent advanced by velocity * dt * steps.
    EXPECT_NEAR(agents.front().location.xy().x(), start.x() + velocity.x() * dt * steps, 1e-9);
    EXPECT_NEAR(agents.front().location.xy().y(), start.y() + velocity.y() * dt * steps, 1e-9);
}

TEST(OperationalDecisionSystem, EveryStepStartsFromTheAgentsItIsGiven)
{
    // The system keeps the generation before as the buffer for the next one. Nothing of it
    // may show through, whether agents came or left since.
    auto geo = flat_square();
    AgentContainer<GenericAgent> agents{};
    NeighborhoodSearch<GenericAgent> neighborhood_search{2.2};
    OperationalDecisionSystem system{std::make_unique<ConstantVelocityModel>()};
    const double dt = 0.5;

    struct Expected {
        GenericAgent::ID id;
        std::string name;
        Point xy;
        Vector velocity;
    };
    std::vector<Expected> expected{};
    const auto add = [&](Point start, Vector velocity) {
        // Too long for the buffer inside a string, so that a stale one would show.
        const auto name = fmt::format("an agent with a name on the heap, number {}", start.x());
        agents.emplace_back(make_agent(*geo, start, velocity, name));
        expected.push_back({agents.back().id, name, start, velocity});
    };
    const auto step_and_check = [&] {
        neighborhood_search.update(agents);
        system.run(dt, 0.0, neighborhood_search, *geo, agents);
        ASSERT_EQ(agents.size(), expected.size());
        for(std::size_t index = 0; index < expected.size(); ++index) {
            auto& [id, name, xy, velocity] = expected[index];
            xy = xy + velocity * dt;
            const auto& agent = agents[index];
            EXPECT_EQ(agent.id, id);
            EXPECT_EQ(agent.name, name);
            EXPECT_NEAR(agent.location.xy().x(), xy.x(), 1e-9);
            EXPECT_NEAR(agent.location.xy().y(), xy.y(), 1e-9);
            const auto& state = std::get<CustomModel::State>(agent.state);
            EXPECT_EQ(state.get<ConstantVelocity>().velocity, velocity);
        }
    };

    add({-3, 0}, {0.1, 0.0});
    add({-2, 0}, {0.0, 0.1});
    add({-1, 0}, {-0.1, 0.0});
    step_and_check();
    step_and_check();

    // More agents than the buffer holds.
    add({1, 0}, {0.0, -0.1});
    add({2, 0}, {0.1, 0.1});
    step_and_check();
    step_and_check();

    // Fewer agents than the buffer holds, and none at the index it had before.
    agents.erase(agents.begin(), agents.begin() + 3);
    expected.erase(expected.begin(), expected.begin() + 3);
    step_and_check();
    step_and_check();
}
