// SPDX-License-Identifier: LGPL-3.0-or-later
#include "GeneralizedCentrifugalForceModel.hpp"
#include "GenericAgent.hpp"
#include "conversion.hpp"

#include <pybind11/cast.h>
#include <pybind11/detail/common.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // IWYU pragma: keep

#include <cstdint>
#include <tuple>

namespace py = pybind11;

void init_agent(py::module_& m)
{
    py::class_<GenericAgent>(m, "Agent")
        .def(
            py::init([](uint64_t journeyId,
                        uint64_t stageId,
                        std::tuple<double, double> position,
                        std::tuple<double, double> orientation,
                        GenericAgent::ModelData model) {
                return GenericAgent(
                    GenericAgent::ID::Invalid,
                    journeyId,
                    stageId,
                    intoPoint(position),
                    intoPoint(orientation),
                    model);
            }),
            py::kw_only(),
            py::arg("journey_id"),
            py::arg("stage_id"),
            py::arg("position"),
            py::arg("orientation"),
            py::arg("model"))
        .def_property_readonly("id", [](const GenericAgent& agent) { return agent.AgentID.GetID(); })
        .def_property_readonly(
            "journey_id", [](const GenericAgent& agent) { return agent.JourneyID.GetID(); })
        .def_property_readonly(
            "stage_id", [](const GenericAgent& agent) { return agent.StageID.GetID(); })
        .def_property_readonly(
            "position", [](const GenericAgent& agent) { return intoTuple(agent.Pos); })
        .def_property_readonly(
            "orientation", [](const GenericAgent& agent) { return intoTuple(agent.Orientation); })
        .def_property(
            "target",
            [](const GenericAgent& agent) { return intoTuple(agent.Target); },
            [](GenericAgent& agent, std::tuple<double, double> target) {
                agent.Target = intoPoint(target);
            })
        .def_property_readonly(
            "model",
            [](GenericAgent& agent) -> auto& { return agent.Model; },
            py::return_value_policy::reference);
}
