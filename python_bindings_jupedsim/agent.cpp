// SPDX-License-Identifier: LGPL-3.0-or-later
#include "conversion.hpp"
#include <Journey.hpp>
#include <Stage.hpp>

#include <GeneralizedCentrifugalForceModel.hpp>
#include <GenericAgent.hpp>
#include <Unreachable.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_agent(py::module_& m)
{
    py::class_<GenericAgent>(m, "Agent")
        .def(
            py::init([](Journey::ID journeyId,
                        BaseStage::ID stageId,
                        std::tuple<double, double> position,
                        std::tuple<double, double> orientation,
                        GenericAgent::Model model) {
                return GenericAgent(
                    GenericAgent::ID::Invalid,
                    journeyId,
                    stageId,
                    intoPoint(position),
                    intoPoint(orientation),
                    model);
            }))
        .def_property_readonly("id", [](const GenericAgent& agent) { return agent.id.getID(); })
        .def_property_readonly(
            "journey_id", [](const GenericAgent& agent) { return agent.journeyId.getID(); })
        .def_property_readonly(
            "stage_id", [](const GenericAgent& agent) { return agent.stageId.getID(); })
        .def_property_readonly("position", [](const GenericAgent& agent) { return agent.pos; })
        .def_property_readonly(
            "orientation", [](const GenericAgent& agent) { return agent.orientation; })
        .def_property(
            "target",
            [](const GenericAgent& agent) { return agent.target; },
            [](GenericAgent& agent, std::tuple<double, double> target) {
                agent.target = intoPoint(target);
            })
        .def_property_readonly("model", [](const GenericAgent& agent) { return agent.model; });
}
