// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <fmt/core.h>

struct CollisionFreeSpeedModelV3Data {
    double StrengthNeighborRepulsion{}; // [rad] max steering authority before upper bound
    double RangeNeighborRepulsion{}; // [m] base interaction range for neighbor influence
    double StrengthGeometryRepulsion{}; // [-] wall repulsion strength
    double RangeGeometryRepulsion{}; // [m] wall repulsion decay length

    double RangeXScale{20.0}; // [-] forward interaction stretch multiplier
    double RangeYScale{8.0}; // [-] lateral interaction stretch multiplier
    double ThetaMaxUpperBound{1.57}; // [rad] hard cap on turn angle per update
    double AgentBuffer{0.0}; // [m] stand-off used in speed law: v=0 at s<=buffer

    double TimeGap{1};
    double V0{1.2};
    double Radius{0.15};
    double HeadingAngle{0.0}; // [rad] persistent relaxed heading state
};

template <>
struct fmt::formatter<CollisionFreeSpeedModelV3Data> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const CollisionFreeSpeedModelV3Data& m, FormatContext& ctx) const
    {
        return fmt::format_to(
            ctx.out(),
            "CollisionFreeSpeedModelV3[StrengthNeighborRepulsion ={}, "
            "RangeNeighborRepulsion ={}, StrengthGeometryRepulsion ={}, RangeGeometryRepulsion ={}, "
            "RangeXScale ={}, RangeYScale ={}, ThetaMaxUpperBound ={}, AgentBuffer ={}, "
            "TimeGap ={}, V0 ={}, Radius ={}, HeadingAngle ={}])",
            m.StrengthNeighborRepulsion,
            m.RangeNeighborRepulsion,
            m.StrengthGeometryRepulsion,
            m.RangeGeometryRepulsion,
            m.RangeXScale,
            m.RangeYScale,
            m.ThetaMaxUpperBound,
            m.AgentBuffer,
            m.TimeGap,
            m.V0,
            m.Radius,
            m.HeadingAngle);
    }
};
