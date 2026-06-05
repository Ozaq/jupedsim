// SPDX-License-Identifier: LGPL-3.0-or-later
#include "CollisionFreeSpeedModelV3.hpp"
#include "CollisionFreeSpeedModelV3Builder.hpp"
#include "CollisionFreeSpeedModelV3Data.hpp"
#include "OperationalModel.hpp"

#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // IWYU pragma: keep

namespace py = pybind11;

void init_collision_free_speed_model_v3(py::module_& m)
{
    py::class_<CollisionFreeSpeedModelV3, OperationalModel>(m, "CollisionFreeSpeedModelV3");
    py::class_<CollisionFreeSpeedModelV3Builder>(m, "CollisionFreeSpeedModelV3Builder")
        .def(py::init<>())
        .def("build", &CollisionFreeSpeedModelV3Builder::Build);

    py::class_<CollisionFreeSpeedModelV3Data>(m, "CollisionFreeSpeedModelV3State")
        .def(
            py::init([](double strengthNeighborRepulsion,
                        double rangeNeighborRepulsion,
                        double strengthGeometryRepulsion,
                        double rangeGeometryRepulsion,
                        double timeGap,
                        double desiredSpeed,
                        double radius,
                        double rangeXScale,
                        double rangeYScale,
                        double thetaMaxUpperBound,
                        double agentBuffer) {
                return CollisionFreeSpeedModelV3Data{
                    .StrengthNeighborRepulsion = strengthNeighborRepulsion,
                    .RangeNeighborRepulsion = rangeNeighborRepulsion,
                    .StrengthGeometryRepulsion = strengthGeometryRepulsion,
                    .RangeGeometryRepulsion = rangeGeometryRepulsion,
                    .RangeXScale = rangeXScale,
                    .RangeYScale = rangeYScale,
                    .ThetaMaxUpperBound = thetaMaxUpperBound,
                    .AgentBuffer = agentBuffer,
                    .TimeGap = timeGap,
                    .V0 = desiredSpeed,
                    .Radius = radius};
            }),
            py::kw_only(),
            py::arg("strength_neighbor_repulsion"),
            py::arg("range_neighbor_repulsion"),
            py::arg("strength_geometry_repulsion"),
            py::arg("range_geometry_repulsion"),
            py::arg("time_gap"),
            py::arg("desired_speed"),
            py::arg("radius"),
            py::arg("range_x_scale") = 20.0,
            py::arg("range_y_scale") = 8.0,
            py::arg("theta_max_upper_bound") = 1.57,
            py::arg("agent_buffer") = 0.0)
        .def_readwrite(
            "strength_neighbor_repulsion",
            &CollisionFreeSpeedModelV3Data::StrengthNeighborRepulsion)
        .def_readwrite(
            "range_neighbor_repulsion", &CollisionFreeSpeedModelV3Data::RangeNeighborRepulsion)
        .def_readwrite(
            "strength_geometry_repulsion",
            &CollisionFreeSpeedModelV3Data::StrengthGeometryRepulsion)
        .def_readwrite(
            "range_geometry_repulsion", &CollisionFreeSpeedModelV3Data::RangeGeometryRepulsion)
        .def_readwrite("range_x_scale", &CollisionFreeSpeedModelV3Data::RangeXScale)
        .def_readwrite("range_y_scale", &CollisionFreeSpeedModelV3Data::RangeYScale)
        .def_readwrite(
            "theta_max_upper_bound", &CollisionFreeSpeedModelV3Data::ThetaMaxUpperBound)
        .def_readwrite("agent_buffer", &CollisionFreeSpeedModelV3Data::AgentBuffer)
        .def_readwrite("time_gap", &CollisionFreeSpeedModelV3Data::TimeGap)
        .def_readwrite("desired_speed", &CollisionFreeSpeedModelV3Data::V0)
        .def_readwrite("radius", &CollisionFreeSpeedModelV3Data::Radius);
}
