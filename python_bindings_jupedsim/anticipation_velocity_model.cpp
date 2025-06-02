// SPDX-License-Identifier: LGPL-3.0-or-later
#include <AnticipationVelocityModelBuilder.hpp>
#include <AnticipationVelocityModelData.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_anticipation_velocity_model(py::module_& m)
{
    py::class_<AnticipationVelocityModelBuilder>(m, "AnticipationVelocityModelBuilder")
        .def(
            py::init<double, double>(),
            py::kw_only(),
            py::arg("pushout_strength"),
            py::arg("rng_seed"))
        .def("build", &AnticipationVelocityModelBuilder::Build);
    py::class_<AnticipationVelocityModelData>(m, "AnticipationVelocityModelState")
        .def_readwrite(
            "strength_neighbor_repulsion",
            &AnticipationVelocityModelData::strengthNeighborRepulsion)
        .def_readwrite(
            "range_neighbor_repulsion", &AnticipationVelocityModelData::rangeNeighborRepulsion)
        .def_readwrite("wall_buffer_distance", &AnticipationVelocityModelData::wallBufferDistance)
        .def_readwrite("anticipation_time", &AnticipationVelocityModelData::anticipationTime)
        .def_readwrite("reaction_time", &AnticipationVelocityModelData::reactionTime)
        .def_readwrite("velocity", &AnticipationVelocityModelData::velocity)
        .def_readwrite("time_gap", &AnticipationVelocityModelData::timeGap)
        .def_readwrite("desired_speed", &AnticipationVelocityModelData::v0)
        .def_readwrite("radius", &AnticipationVelocityModelData::radius);
}
