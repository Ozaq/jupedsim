// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "FormatAny.hpp"

#include <any>
#include <type_traits>
#include <utility>

/// Type-erased per-agent state for CustomModel implementations.
///
/// CustomModelData lets a custom operational model store model-specific state in
/// GenericAgent::model without adding a new built-in model data type. The payload is stored by
/// value in std::any so custom state follows the same ownership model as built-in agent model data.
///
/// Payload types must be copy-constructible. GenericAgent values are copied by parts of the
/// simulation, for example by neighborhood queries, so custom state must remain valid under normal
/// value-copy semantics.
///
/// Access is runtime-typed: Get<T>() must use the exact stored type T. A type mismatch throws
/// std::bad_any_cast.
///
/// Formatting is best-effort. If the payload has a fmt formatter, that is used. Otherwise, if it
/// provides a public ToString() callable on const T& and returning std::string, std::string_view,
/// or const char*, ToString() is used. If neither is available, formatting falls back to a
/// diagnostic <type@address> representation.
struct CustomModelDataTag;
using CustomModelData = AnyHolder<CustomModelDataTag>;
