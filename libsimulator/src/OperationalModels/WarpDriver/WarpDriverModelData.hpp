// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <fmt/core.h>

struct WarpDriverModelData {
    double Radius{0.15};
    double V0{1.2};
    double StuckTime{0.0}; // elapsed time since anchor was set
    double AnchorX{0.0}; // position when stuck tracking began
    double AnchorY{0.0};
    double DetourTime{0.0}; // remaining time in detour mode
    int DetourSide{1}; // +1 = left, -1 = right of desired direction
};

template <>
struct fmt::formatter<WarpDriverModelData> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const WarpDriverModelData& m, FormatContext& ctx) const
    {
        return fmt::format_to(ctx.out(), "WarpDriver[Radius ={}, V0 ={}]", m.Radius, m.V0);
    }
};
