// SPDX-License-Identifier: LGPL-3.0-or-later
#include "conversion.hpp"
#include <CollisionGeometry.hpp>
#include <GeometryBuilder.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_geometry(py::module_& m)
{
    py::class_<CollisionGeometry>(m, "Geometry")
        .def(
            "boundary",
            [](const CollisionGeometry& geo) { return std::get<0>(geo.AccessibleArea()); })
        .def("holes", [](const CollisionGeometry& geo) {
            return std::get<1>(geo.AccessibleArea());
        });
    py::class_<GeometryBuilder>(m, "GeometryBuilder")
        .def(py::init<>())
        .def(
            "add_accessible_area",
            [](GeometryBuilder& builder, const std::vector<std::tuple<double, double>>& points) {
                builder.AddAccessibleArea(intoPoints(points));
            })
        .def("exclude_from_accessible_area", &GeometryBuilder::ExcludeFromAccessibleArea)
        .def("build", &GeometryBuilder::Build);
}
