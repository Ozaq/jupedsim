// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

/// @file FormatAny.hpp
/// Helpers for printing types stored in std::any.
/// The goal is to allow printing types stored in std::any where the concrete type is known
/// and is fixed.

#include <fmt/core.h>

#include <any>
#include <memory>
#include <type_traits>

/// Format function type, returned by `makeFormatFn<T>()`.
using FormatFn = fmt::format_context::iterator (*)(const std::any& value, fmt::format_context& ctx);

/// Creates a formatter function for values stored as std::any.
///
/// @tparam T Original value type used to initialize the std::any.
/// @return A function pointer that formats a std::any containing std::decay_t<T>.
/// @throws std::bad_any_cast if the returned function is called with a std::any that does not hold
///         std::decay_t<T>.
/// @note Exceptions thrown by a custom fmt formatter for std::decay_t<T> are propagated.
/// @note If std::decay_t<T> is fmt-formattable, the contained value is formatted directly using
///       fmt::format_to(). Otherwise, the fallback output contains the implementation-defined type
///       name from std::type_info::name() and the address of the contained object.
/// @warning The fallback address is only diagnostic. It is valid only while the std::any is alive
///          and unchanged, and must not be treated as a stable object identity.
template <typename T>
auto makeFormatFn()
{
    using Stored = std::decay_t<T>;
    if constexpr(fmt::is_formattable<Stored, char>::value) {
        return [](const std::any& a, fmt::format_context& ctx) {
            return fmt::format_to(ctx.out(), "{}", std::any_cast<const Stored&>(a));
        };
    } else {
        return [](const std::any& a, fmt::format_context& ctx) {
            const auto& value = std::any_cast<const Stored&>(a);
            const auto addr = static_cast<const void*>(std::addressof(value));
            return fmt::format_to(ctx.out(), "<{}@{}>", a.type().name(), addr);
        };
    }
}
