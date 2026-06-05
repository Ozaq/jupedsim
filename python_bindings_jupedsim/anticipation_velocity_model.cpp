// SPDX-License-Identifier: LGPL-3.0-or-later
#include "AnticipationVelocityModel.hpp"
#include "AnticipationVelocityModelBuilder.hpp"
#include "AnticipationVelocityModelData.hpp"
#include "OperationalModel.hpp"

#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // IWYU pragma: keep

namespace py = pybind11;

void init_anticipation_velocity_model(py::module_& m)
{
    py::class_<AnticipationVelocityModel, OperationalModel>(m, "AnticipationVelocityModel");
    py::class_<AnticipationVelocityModelBuilder>(m, "AnticipationVelocityModelBuilder")
        .def(
            py::init<double, double>(),
            py::kw_only(),
            py::arg("pushout_strength"),
            py::arg("rng_seed"))
        .def("build", &AnticipationVelocityModelBuilder::Build);
    py::class_<AnticipationVelocityModelData>(m, "AnticipationVelocityModelState")
        .def_static("_defaults", []() { return AnticipationVelocityModelData{}; })
        .def(
            py::init([](double strengthNeighborRepulsion,
                        double rangeNeighborRepulsion,
                        double wallBufferDistance,
                        double anticipationTime,
                        double reactionTime,
                        double timeGap,
                        double desiredSpeed,
                        double radius) {
                return AnticipationVelocityModelData{
                    .StrengthNeighborRepulsion = strengthNeighborRepulsion,
                    .RangeNeighborRepulsion = rangeNeighborRepulsion,
                    .WallBufferDistance = wallBufferDistance,
                    .AnticipationTime = anticipationTime,
                    .ReactionTime = reactionTime,
                    .TimeGap = timeGap,
                    .V0 = desiredSpeed,
                    .Radius = radius};
            }),
            py::kw_only(),
            py::arg("strength_neighbor_repulsion"),
            py::arg("range_neighbor_repulsion"),
            py::arg("wall_buffer_distance"),
            py::arg("anticipation_time"),
            py::arg("reaction_time"),
            py::arg("time_gap"),
            py::arg("desired_speed"),
            py::arg("radius"))
        .def_readwrite(
            "strength_neighbor_repulsion",
            &AnticipationVelocityModelData::StrengthNeighborRepulsion)
        .def_readwrite(
            "range_neighbor_repulsion", &AnticipationVelocityModelData::RangeNeighborRepulsion)
        .def_readwrite("wall_buffer_distance", &AnticipationVelocityModelData::WallBufferDistance)
        .def_readwrite("anticipation_time", &AnticipationVelocityModelData::AnticipationTime)
        .def_readwrite("reaction_time", &AnticipationVelocityModelData::ReactionTime)
        .def_readwrite("velocity", &AnticipationVelocityModelData::Velocity)
        .def_readwrite("time_gap", &AnticipationVelocityModelData::TimeGap)
        .def_readwrite("desired_speed", &AnticipationVelocityModelData::V0)
        .def_readwrite("radius", &AnticipationVelocityModelData::Radius);
}
