// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "FormatAny.hpp"

#include <any>
#include <utility>

class UserDefinedModelData
{
private:
    std::any value{};
    FormatFn format{};

public:
    template <typename T>
        requires(!std::is_same_v<std::decay_t<T>, UserDefinedModelData>)
    UserDefinedModelData(T&& value) : value(std::forward<T>(value)), format(makeFormatFn<T>())
    {
    }

    template <typename T>
    T& Get()
    {
        return std::any_cast<T&>(value);
    }

    template <typename T>
    const T& Get() const
    {
        return std::any_cast<const T&>(value);
    }

    template <typename T>
    void Set(T&& newValue)
    {
        using Stored = std::decay_t<T>;
        std::any_cast<Stored&>(value) = std::forward<T>(newValue);
    }

    friend struct fmt::formatter<UserDefinedModelData>;
};

template <>
struct fmt::formatter<UserDefinedModelData> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    auto format(const UserDefinedModelData& value, fmt::format_context& ctx) const
    {
        return value.format(value.value, ctx);
    }
};
