// SPDX-License-Identifier: LGPL-3.0-or-later
#include <CollisionFreeSpeedModelV2Builder.hpp>
#include <CollisionFreeSpeedModelV2Data.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_collision_free_speed_model_v2(py::module_& m)
{
    py::class_<CollisionFreeSpeedModelV2Builder>(m, "CollisionFreeSpeedModelV2Builder")
        .def(py::init<>())
        .def("build", &CollisionFreeSpeedModelV2Builder::Build);

    py::class_<CollisionFreeSpeedModelV2Data>(m, "CollisionFreeSpeedModelV2State")
        .def_readwrite(
            "strength_neighbor_repulsion",
            &CollisionFreeSpeedModelV2Data::strengthNeighborRepulsion)
        .def_readwrite(
            "range_neighbor_repulsion", &CollisionFreeSpeedModelV2Data::rangeNeighborRepulsion)
        .def_readwrite(
            "strength_geometry_repulsion",
            &CollisionFreeSpeedModelV2Data::strengthGeometryRepulsion)
        .def_readwrite(
            "range_geometry_repulsion", &CollisionFreeSpeedModelV2Data::rangeGeometryRepulsion)
        .def_readwrite("time_gap", &CollisionFreeSpeedModelV2Data::timeGap)
        .def_readwrite("desired_speed", &CollisionFreeSpeedModelV2Data::v0)
        .def_readwrite("radius", &CollisionFreeSpeedModelV2Data::radius);
}
