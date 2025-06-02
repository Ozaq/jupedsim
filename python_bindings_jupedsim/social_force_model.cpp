// SPDX-License-Identifier: LGPL-3.0-or-later
#include <SocialForceModelBuilder.hpp>
#include <SocialForceModelData.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_social_force_model(py::module_& m)
{
    py::class_<SocialForceModelBuilder>(m, "SocialForceModelBuilder")
        .def(py::init<double, double>(), py::kw_only(), py::arg("body_force"), py::arg("friction"))
        .def("build", &SocialForceModelBuilder::Build);
    py::class_<SocialForceModelData>(m, "SocialForceModelState")
        .def_readwrite("velocity", &SocialForceModelData::velocity)
        .def_readwrite("mass", &SocialForceModelData::mass)
        .def_readwrite("desired_speed", &SocialForceModelData::desiredSpeed)
        .def_readwrite("reaction_time", &SocialForceModelData::reactionTime)
        .def_readwrite("agent_scale", &SocialForceModelData::agentScale)
        .def_readwrite("obstacle_scale", &SocialForceModelData::obstacleScale)
        .def_readwrite("force_distance", &SocialForceModelData::forceDistance)
        .def_readwrite("radius", &SocialForceModelData::radius);
}
