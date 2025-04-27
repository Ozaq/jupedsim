// SPDX-License-Identifier: LGPL-3.0-or-later
#include "conversion.hpp"
#include "wrapper.hpp"

#include <Unreachable.hpp>
#include <jupedsim/jupedsim.h>

#include <GeneralizedCentrifugalForceModel.hpp>
#include <GenericAgent.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_agent(py::module_& m)
{
    py::class_<GenericAgent>(m, "Agent")
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
