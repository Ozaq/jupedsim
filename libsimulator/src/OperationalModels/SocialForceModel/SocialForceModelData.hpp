// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Point.hpp"

#include <fmt/core.h>
struct SocialForceModelData {
    Point Velocity{}; // v
    double Mass{80.0}; // m
    double DesiredSpeed{0.8}; // v0
    double ReactionTime{0.5}; // tau
    double AgentScale{2000.0}; // A for other agents
    double ObstacleScale{2000.0}; // A for obstacles
    double ForceDistance{0.08}; // B
    double Radius{0.3}; // r
};

template <>
struct fmt::formatter<SocialForceModelData> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const SocialForceModelData& m, FormatContext& ctx) const
    {
        return fmt::format_to(
            ctx.out(),
            "SFM[Velocity ={}, m={}, V0 ={}, Tau ={}, A_ped={}, A_obst={}, B={}, r={}])",
            m.Velocity,
            m.Mass,
            m.DesiredSpeed,
            m.ReactionTime,
            m.AgentScale,
            m.ObstacleScale,
            m.ForceDistance,
            m.Radius);
    }
};
