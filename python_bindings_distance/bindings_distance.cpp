// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include <pybind11/pybind11.h>

#include <common_types.hpp>
#include <grid.hpp>
#include <raster_map.hpp>

namespace py = pybind11;

using Real = double;
using Point = distance::Point<Real>;
using LineSegment = distance::LineSegment<Real>;
using Polygon = distance::Polygon<Real>;
using AABB = distance::AABB<Real>;
using RasterMap = distance::RasterMap<Real, Real>;
using Grid = RasterMap::GridT;

PYBIND11_MODULE(py_distance, m)
{
    py::class_<Point>(m, "Point")
        .def(py::init<Real, Real>())
        .def_readwrite("x", &Point::x)
        .def_readwrite("y", &Point::y)
        .def(
            "__getitem__",
            [](Point& p, int index) {
                switch(index) {
                    case 0:
                        return p.x;
                    case 1:
                        return p.y;
                }
                throw py::index_error{};
            })
        .def("__setitem__", [](Point& p, int index, Real value) {
            switch(index) {
                case 0:
                    p.x = value;
                    return;
                case 1:
                    p.y = value;
                    return;
            }
            throw py::index_error{};
        });
    py::class_<LineSegment>(m, "LineSegment")
        .def(py::init<Point, Point>())
        .def_readwrite("p1", &LineSegment::p1)
        .def_readwrite("p2", &LineSegment::p2)
        .def(
            "__getitem__",
            [](const LineSegment& ls, int index) {
                switch(index) {
                    case 0:
                        return ls.p1;
                    case 1:
                        return ls.p2;
                }
                throw py::index_error{};
            })
        .def("__setitem__", [](LineSegment& ls, int index, Point value) {
            switch(index) {
                case 0:
                    ls.p1 = value;
                    return;
                case 1:
                    ls.p2 = value;
                    return;
            }
            throw py::index_error{};
        });
    py::class_<AABB>(m, "AABB")
        .def(py::init<Point, Point>())
        .def("width", &AABB::Width)
        .def("height", &AABB::Height)
        .def("lower_left", &AABB::LowerLeft);
    py::class_<Grid>(m, "Grid")
        .def(
            py::init<uint64_t, uint64_t, Real>(),
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
    py::class_<RasterMap>(m, "RasterMap")
        .def(py::init<AABB, Real>())
        .def("mark_point", [](RasterMap& rm, Point pt, Real value) { rm.MarkPoint(pt, value); })
        .def(
            "mark_line_segment",
            [](RasterMap& rm, LineSegment ls, Real value) { rm.MarkLineSegment(ls, value); })
        .def("mark_polygon", &RasterMap::MarkPolygon)
        .def("__getitem__", [](const RasterMap& rm, Point pt) { return rm.At(pt); })
        .def_property_readonly("grid", [](const RasterMap& rm) { return rm.Grid(); });
    py::implicitly_convertible<std::tuple<Real, Real>, Point>();
    py::implicitly_convertible<std::tuple<Point, Point>, AABB>();
}
