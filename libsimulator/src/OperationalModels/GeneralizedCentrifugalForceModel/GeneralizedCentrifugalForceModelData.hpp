// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Point.hpp"

#include <fmt/core.h>
struct GeneralizedCentrifugalForceModelData {
    double Speed{};
    Point E0{};
    int OrientationDelay{};
    double Mass{1.0};
    double Tau{0.5};
    double V0{1.2};
    double Av{1.0};
    double AMin{0.2};
    double BMin{0.2};
    double BMax{0.4};
};

template <>
struct fmt::formatter<GeneralizedCentrifugalForceModelData> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const GeneralizedCentrifugalForceModelData& m, FormatContext& ctx) const
    {
        return fmt::format_to(ctx.out(), "GCFM[Speed ={}])", m.Speed);
    }
};
