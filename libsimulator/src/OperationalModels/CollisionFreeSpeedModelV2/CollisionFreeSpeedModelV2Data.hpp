// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <fmt/core.h>

struct CollisionFreeSpeedModelV2Data {
    double StrengthNeighborRepulsion{8.0};
    double RangeNeighborRepulsion{0.1};
    double StrengthGeometryRepulsion{5.0};
    double RangeGeometryRepulsion{0.02};

    double TimeGap{1};
    double V0{1.2};
    double Radius{0.2};
};

template <>
struct fmt::formatter<CollisionFreeSpeedModelV2Data> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const CollisionFreeSpeedModelV2Data& m, FormatContext& ctx) const
    {
        return fmt::format_to(
            ctx.out(),
            "CollisionFreeSpeedModelV2[StrengthNeighborRepulsion ={}, "
            "RangeNeighborRepulsion ={}, StrengthGeometryRepulsion ={}, RangeGeometryRepulsion ={}, "
            "TimeGap ={}, V0 ={}, Radius ={}])",
            m.StrengthNeighborRepulsion,
            m.RangeNeighborRepulsion,
            m.StrengthGeometryRepulsion,
            m.RangeGeometryRepulsion,
            m.TimeGap,
            m.V0,
            m.Radius);
    }
};
