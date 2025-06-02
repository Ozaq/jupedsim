// SPDX-License-Identifier: LGPL-3.0-or-later
#include "conversion.hpp"
#include <Journey.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_transition(py::module_& m)
{
    py::class_<TransitionDescription>(m, "Transition")
        .def_static(
            "create_fixed_transition",
            [](BaseStage::ID stageId) { return FixedTransitionDescription(stageId); })
        .def_static(
            "create_round_robin_transition",
            [](const std::vector<std::tuple<BaseStage::ID, uint64_t>>& stageWeights) {
                return RoundRobinTransitionDescription(stageWeights);
            })
        .def_static(
            "create_least_targeted_transition", [](const std::vector<BaseStage::ID>& stages) {
                return LeastTargetedTransitionDescription(stages);
            });
}
