// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "OperationalModel.hpp"

/// Base class for operational models implemented outside libsimulator.
///
/// Derive from this class when a model is not part of the built-in model set. The derived model
/// still implements the pure virtual interface inherited from OperationalModel:
/// ComputeNewPosition(), ApplyUpdate(), CheckModelConstraint(), and Clone(). This class only fixes
/// the model type to OperationalModelType::USER_DEFINED so custom models do not need to repeat that
/// boilerplate.
///
/// Per-agent custom state should be stored in GenericAgent::model as UserDefinedModelData. Custom
/// model updates should be returned from ComputeNewPosition() as UserDefinedModelUpdate. Both types
/// keep their payload in std::any, so model implementations must agree on the concrete stored types
/// and retrieve them with the typed accessors.
///
/// @code
/// class MyModel : public UserDefinedModel
/// {
/// public:
///     OperationalModelUpdate ComputeNewPosition(
///         double dT,
///         const GenericAgent& agent,
///         const CollisionGeometry& geometry,
///         const NeighborhoodSearch<GenericAgent>& neighborhoodSearch) const override;
///
///     void ApplyUpdate(const OperationalModelUpdate& update, GenericAgent& agent) const override;
///
///     void CheckModelConstraint(
///         const GenericAgent& agent,
///         const NeighborhoodSearch<GenericAgent>& neighborhoodSearch,
///         const CollisionGeometry& geometry) const override;
///
///     std::unique_ptr<OperationalModel> Clone() const override;
/// };
/// @endcode
///
/// @note UserDefinedModel is still abstract. It cannot be instantiated directly.
/// @warning std::any payloads are type-erased. A mismatched accessor type throws std::bad_any_cast.
class UserDefinedModel : public OperationalModel
{
public:
    UserDefinedModel() = default;
    ~UserDefinedModel() override = default;

    OperationalModelType Type() const override { return OperationalModelType::USER_DEFINED; }
};
