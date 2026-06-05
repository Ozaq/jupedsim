// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once
#include "AnticipationVelocityModelData.hpp"
#include "CollisionFreeSpeedModelData.hpp"
#include "CollisionFreeSpeedModelV2Data.hpp"
#include "CollisionFreeSpeedModelV3Data.hpp"
#include "GeneralizedCentrifugalForceModelData.hpp"
#include "OperationalModel.hpp"
#include "Point.hpp"
#include "SocialForceModelData.hpp"
#include "UniqueID.hpp"
#include "WarpDriverModelData.hpp"

#include <fmt/core.h>

#include <utility>
#include <variant>
class Journey;
class BaseStage;

struct GenericAgent {
    using ID = jps::UniqueID<GenericAgent>;
    ID AgentID{};

    jps::UniqueID<Journey> JourneyID{jps::UniqueID<Journey>::Invalid};
    jps::UniqueID<BaseStage> StageID{jps::UniqueID<BaseStage>::Invalid};

    // This is evaluated by the "operational level"
    Point Destination{};
    Point Target{};

    // Agent fields common for all models
    Point Pos{};
    Point Orientation{};

    using ModelData = std::variant<
        GeneralizedCentrifugalForceModelData,
        CollisionFreeSpeedModelData,
        CollisionFreeSpeedModelV2Data,
        CollisionFreeSpeedModelV3Data,
        AnticipationVelocityModelData,
        SocialForceModelData,
        WarpDriverModelData>;
    ModelData Model{};

    GenericAgent(
        ID id_,
        jps::UniqueID<Journey> journeyId_,
        jps::UniqueID<BaseStage> stageId_,
        Point pos_,
        Point orientation_,
        ModelData model_)
        : AgentID(id_ != ID::Invalid ? id_ : ID{})
        , JourneyID(journeyId_)
        , StageID(stageId_)
        , Target(pos_)
        , Pos(pos_)
        , Orientation(orientation_)
        , Model(std::move(model_))
    {
    }
};
template <>
struct fmt::formatter<GenericAgent> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const GenericAgent& agent, FormatContext& ctx) const
    {
        return std::visit(
            [&ctx, &agent](const auto& m) {
                return fmt::format_to(
                    ctx.out(),
                    "Agent[id={}, journey={}, stage={}, destination={}, waypoint={}, pos={}, "
                    "orientation={}, model={})",
                    agent.AgentID,
                    agent.JourneyID,
                    agent.StageID,
                    agent.Destination,
                    agent.Target,
                    agent.Pos,
                    agent.Orientation,
                    m);
            },
            agent.Model);
    }
};
