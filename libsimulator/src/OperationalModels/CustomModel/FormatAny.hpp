// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

/// @file FormatAny.hpp
/// Helpers for formatting values stored in std::any.

#include <fmt/core.h>

#include <any>
#include <concepts>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>

using FormatFn = fmt::format_context::iterator (*)(const std::any& value, fmt::format_context& ctx);

template <typename T>
concept ToStringResult = std::same_as<std::remove_cvref_t<T>, std::string> ||
                         std::same_as<std::remove_cvref_t<T>, std::string_view> ||
                         std::same_as<std::remove_cvref_t<T>, const char*>;

template <typename T>
concept HasToString = requires(const T& value) {
    { value.ToString() } -> ToStringResult;
};

template <typename T>
auto makeFormatFn()
{
    using Stored = std::decay_t<T>;
    if constexpr(fmt::formattable<Stored, char>) {
        return [](const std::any& any, fmt::format_context& ctx) {
            return fmt::format_to(ctx.out(), "{}", std::any_cast<const Stored&>(any));
        };
    } else if constexpr(HasToString<Stored>) {
        return [](const std::any& any, fmt::format_context& ctx) {
            const auto& value = std::any_cast<const Stored&>(any);
            return fmt::format_to(ctx.out(), "{}", value.ToString());
        };
    } else {
        return [](const std::any& any, fmt::format_context& ctx) {
            const auto& value = std::any_cast<const Stored&>(any);
            const auto addr = static_cast<const void*>(std::addressof(value));
            return fmt::format_to(ctx.out(), "<{}@{}>", any.type().name(), addr);
        };
    }
}
template <typename Tag>
class AnyHolder
{
private:
    std::any value{};
    FormatFn format{};

public:
    template <typename T>
        requires(!std::is_same_v<std::decay_t<T>, AnyHolder>)
    AnyHolder(T&& value) : value(std::forward<T>(value)), format(makeFormatFn<T>())
    {
        using Stored = std::decay_t<T>;
        static_assert(
            std::is_copy_constructible_v<Stored>, "AnyHolder payloads must be copy-constructible");
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

    friend struct fmt::formatter<AnyHolder>;
};

template <typename Tag>
struct fmt::formatter<AnyHolder<Tag>> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    auto format(const AnyHolder<Tag>& value, fmt::format_context& ctx) const
    {
        return value.format(value.value, ctx);
    }
};
