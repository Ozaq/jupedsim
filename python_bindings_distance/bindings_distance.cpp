// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include <pybind11/pybind11.h>

#include <grid.hpp>

namespace py = pybind11;

using Grid = distance::Grid<double>;

PYBIND11_MODULE(py_distance, m)
{
    py::class_<Grid>(m, "Grid")
        .def(
            py::init<uint64_t, uint64_t, double>(),
            py::kw_only(),
            py::arg("width"),
            py::arg("height"),
            py::arg("default_value"))
        .def("width", [](const Grid& grid) { return grid.Width(); })
        .def("height", [](const Grid& grid) { return grid.Height(); })
        .def(
            "__getitem__",
            [](const Grid& grid, std::tuple<size_t, size_t> idx) {
                return grid.At({std::get<0>(idx), std::get<1>(idx)});
            })
        .def("__str__", [](const Grid& grid) { return grid.DumpCSV(); });
}
