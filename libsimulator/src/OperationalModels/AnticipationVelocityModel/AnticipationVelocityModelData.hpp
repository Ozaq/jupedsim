// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Point.hpp"

#include <fmt/core.h>

struct AnticipationVelocityModelData {
    double StrengthNeighborRepulsion{8.0};
    double RangeNeighborRepulsion{0.1};
    double WallBufferDistance{0.1}; // buff distance of agent to wall
    double AnticipationTime{1.0}; // anticipation time
    double ReactionTime{0.3}; // reaction time to update direction
    Point Velocity{};
    double TimeGap{1.06};
    double V0{1.2};
    double Radius{0.2};
};

template <>
struct fmt::formatter<AnticipationVelocityModelData> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const AnticipationVelocityModelData& m, FormatContext& ctx) const
    {
        return fmt::format_to(
            ctx.out(),
            "AnticipationVelocityModel[StrengthNeighborRepulsion ={}, "
            "RangeNeighborRepulsion ={}, WallBufferDistance ={}, "
            "TimeGap ={}, V0 ={}, Radius ={}, ReactionTime ={}, AnticipationTime ={}, Velocity ={}])",
            m.StrengthNeighborRepulsion,
            m.RangeNeighborRepulsion,
            m.WallBufferDistance,
            m.TimeGap,
            m.V0,
            m.Radius,
            m.ReactionTime,
            m.AnticipationTime,
            m.Velocity);
    }
};
