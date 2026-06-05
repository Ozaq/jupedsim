// SPDX-License-Identifier: LGPL-3.0-or-later
#include "CollisionFreeSpeedModelV2.hpp"
#include "CollisionFreeSpeedModelV2Builder.hpp"
#include "CollisionFreeSpeedModelV2Data.hpp"
#include "OperationalModel.hpp"

#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // IWYU pragma: keep

namespace py = pybind11;

void init_collision_free_speed_model_v2(py::module_& m)
{
    py::class_<CollisionFreeSpeedModelV2, OperationalModel>(m, "CollisionFreeSpeedModelV2");
    py::class_<CollisionFreeSpeedModelV2Builder>(m, "CollisionFreeSpeedModelV2Builder")
        .def(py::init<>())
        .def("build", &CollisionFreeSpeedModelV2Builder::Build);

    py::class_<CollisionFreeSpeedModelV2Data>(m, "CollisionFreeSpeedModelV2State")
        .def_static("_defaults", []() { return CollisionFreeSpeedModelV2Data{}; })
        .def(
            py::init([](double strengthNeighborRepulsion,
                        double rangeNeighborRepulsion,
                        double strengthGeometryRepulsion,
                        double rangeGeometryRepulsion,
                        double timeGap,
                        double desiredSpeed,
                        double radius) {
                return CollisionFreeSpeedModelV2Data{
                    .StrengthNeighborRepulsion = strengthNeighborRepulsion,
                    .RangeNeighborRepulsion = rangeNeighborRepulsion,
                    .StrengthGeometryRepulsion = strengthGeometryRepulsion,
                    .RangeGeometryRepulsion = rangeGeometryRepulsion,
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
            py::arg("radius"))
        .def_readwrite(
            "strength_neighbor_repulsion",
            &CollisionFreeSpeedModelV2Data::StrengthNeighborRepulsion)
        .def_readwrite(
            "range_neighbor_repulsion", &CollisionFreeSpeedModelV2Data::RangeNeighborRepulsion)
        .def_readwrite(
            "strength_geometry_repulsion",
            &CollisionFreeSpeedModelV2Data::StrengthGeometryRepulsion)
        .def_readwrite(
            "range_geometry_repulsion", &CollisionFreeSpeedModelV2Data::RangeGeometryRepulsion)
        .def_readwrite("time_gap", &CollisionFreeSpeedModelV2Data::TimeGap)
        .def_readwrite("desired_speed", &CollisionFreeSpeedModelV2Data::V0)
        .def_readwrite("radius", &CollisionFreeSpeedModelV2Data::Radius);
}
