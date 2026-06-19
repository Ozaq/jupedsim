#include "LineSegment.hpp"

#include "type_casters.hpp"

#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_linesegment(py::module_& m)
{
    py::class_<LineSegment>(m, "LineSegment")
        .def(py::init<>())
        .def(py::init<Point, Point>(), py::kw_only(), py::arg("p1"), py::arg("p2"))
        .def_readwrite("p1", &LineSegment::p1)
        .def_readwrite("p2", &LineSegment::p2)
        .def("shortest_point", &LineSegment::ShortestPoint, py::arg("p"))
        .def("dist_to", &LineSegment::DistTo, py::arg("p"));
}
