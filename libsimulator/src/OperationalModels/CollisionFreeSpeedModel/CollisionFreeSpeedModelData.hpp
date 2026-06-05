// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <fmt/core.h>

struct CollisionFreeSpeedModelData {
    double TimeGap{1};
    double V0{1.2};
    double Radius{0.2};
};

template <>
struct fmt::formatter<CollisionFreeSpeedModelData> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const CollisionFreeSpeedModelData& m, FormatContext& ctx) const
    {
        return fmt::format_to(
            ctx.out(),
            "CollisionFreeSpeedModel[TimeGap ={}, V0 ={}, Radius ={}])",
            m.TimeGap,
            m.V0,
            m.Radius);
    }
};
