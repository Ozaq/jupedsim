// SPDX-License-Identifier: LGPL-3.0-or-later
#include <Journey.hpp>
#include <Stage.hpp>

#include <map>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using JourneyDesc = std::map<BaseStage::ID, TransitionDescription>;

void init_journey(py::module_& m)
{
    py::class_<JourneyDesc>(m, "JourneyDescription")
        .def(py::init([]() { return std::make_unique<JourneyDesc>(); }))
        .def(py::init([](const std::vector<BaseStage::ID>& ids) {
            auto desc = std::make_unique<JourneyDesc>();
            for(auto id : ids) {
                (*desc)[id] = NonTransitionDescription{};
            }
            return desc;
        }))
        .def(
            "add",
            [](JourneyDesc& desc, BaseStage::ID id) {
                desc.insert(std::make_pair(id, NonTransitionDescription{}));
            })
        .def(
            "add",
            [](JourneyDesc& desc, const std::vector<BaseStage::ID>& ids) {
                for(const auto& id : ids) {
                    desc.insert(std::make_pair(id, NonTransitionDescription{}));
                }
            })
        .def(
            "set_transition_for_stage",
            [](JourneyDesc& desc, BaseStage::ID stageId, TransitionDescription& transition) {
                auto iter = desc.find(stageId);
                if(iter == std::end(desc)) {
                    throw std::runtime_error(
                        fmt::format(
                            "Could not set transition for given stage id {}. Stage not found.",
                            stageId));
                }
                iter->second = transition;
            });
}
