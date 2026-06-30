// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "FormatAny.hpp"

#include <any>
#include <type_traits>
#include <utility>

/// Type-erased update payload for CustomModel implementations.
///
/// CustomModelUpdate is returned from CustomModel::ComputeNewPosition and later consumed by
/// CustomModel::ApplyUpdate. Keeping update data separate from CustomModelData preserves JuPedSim's
/// existing two-phase operational update flow: first compute all updates from the current state,
/// then apply them.
///
/// Like CustomModelData, the payload is stored by value in std::any. Payload types must be
/// copy-constructible, access through Get<T>() requires the exact stored type, and mismatches throw
/// std::bad_any_cast.
///
/// Formatting follows the same rules as CustomModelData: fmt formatter first, then public
/// const-callable ToString() returning std::string, std::string_view, or const char*, then a
/// diagnostic <type@address> fallback.
using CustomModelUpdate = AnyHolder<struct CustomModelUpdateTag>;
