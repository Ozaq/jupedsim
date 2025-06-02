// SPDX-License-Identifier: LGPL-3.0-or-later
#include <GeneralizedCentrifugalForceModelBuilder.hpp>
#include <GeneralizedCentrifugalForceModelData.hpp>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_generalized_centrifugal_force_model(py::module_& m)
{
    py::class_<GeneralizedCentrifugalForceModelBuilder>(
        m, "GeneralizedCentrifugalForceModelBuilder")
        .def(
            py::init<double, double, double, double, double, double, double, double>(),
            py::kw_only(),
            py::arg("strength_neighbor_repulsion"),
            py::arg("strength_geometry_repulsion"),
            py::arg("max_neighbor_interaction_distance"),
            py::arg("max_geometry_interaction_distance"),
            py::arg("max_neighbor_interpolation_distance"),
            py::arg("max_geometry_interpolation_distance"),
            py::arg("max_neighbor_repulsion_force"),
            py::arg("max_geometry_repulsion_force"))
        .def("build", &GeneralizedCentrifugalForceModelBuilder::Build);
    py::class_<GeneralizedCentrifugalForceModelData>(m, "GeneralizedCentrifugalForceModelState")
        .def(py::init(), py::kw_only())
        .def_readwrite("speed", &GeneralizedCentrifugalForceModelData::speed)
        .def_readwrite("desired_orientation", &GeneralizedCentrifugalForceModelData::e0)
        .def_readwrite("orientation_delay", &GeneralizedCentrifugalForceModelData::orientationDelay)
        .def_readwrite("mass", &GeneralizedCentrifugalForceModelData::mass)
        .def_readwrite("tau", &GeneralizedCentrifugalForceModelData::tau)
        .def_readwrite("desired_speed", &GeneralizedCentrifugalForceModelData::v0)
        .def_readwrite("a_v", &GeneralizedCentrifugalForceModelData::Av)
        .def_readwrite("a_min", &GeneralizedCentrifugalForceModelData::AMin)
        .def_readwrite("b_min", &GeneralizedCentrifugalForceModelData::BMin)
        .def_readwrite("b_max", &GeneralizedCentrifugalForceModelData::BMax);
}
