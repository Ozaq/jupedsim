// SPDX-License-Identifier: LGPL-3.0-or-later
#include <Journey.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
void init_transition(py::module_& m)
{
    py::class_<TransitionDescription>(m, "Transition")
        .def_static(
            "create_fixed_transition",
            [](uint64_t stageId) { return FixedTransitionDescription(stageId); })
        .def_static(
            "create_round_robin_transition",
            [](const std::vector<std::tuple<uint64_t, uint64_t>>& stageWeights) {
                std::vector<std::tuple<BaseStage::ID, uint64_t>> mapped{};
                mapped.reserve(stageWeights.size());
                std::transform(
                    std::begin(stageWeights),
                    std::end(stageWeights),
                    std::back_inserter(mapped),
                    [](const auto& t) {
                        return std::make_tuple(BaseStage::ID(std::get<0>(t)), std::get<1>(t));
                    });
                return RoundRobinTransitionDescription(mapped);
            })
        .def_static("create_least_targeted_transition", [](const std::vector<uint64_t>& stages) {
            std::vector<BaseStage::ID> mapped{};
            mapped.reserve(stages.size());
            std::transform(
                std::begin(stages),
                std::end(stages),
                std::back_inserter(mapped),
                [](const auto& t) { return BaseStage::ID(t); });
            return LeastTargetedTransitionDescription(mapped);
        });
}
