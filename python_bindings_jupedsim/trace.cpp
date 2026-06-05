// SPDX-License-Identifier: LGPL-3.0-or-later

#include "Timing.hpp"
#include "Tracing.hpp"

#include <fmt/core.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_trace(py::module_& m)
{
    // See https://stackoverflow.com/a/50592578 for how to define a Singleton with pybind11
    py::class_<Profiler, std::unique_ptr<Profiler, py::nodelete>>(m, "Profiler")
        .def(py::init([]() {
            return std::unique_ptr<Profiler, py::nodelete>(&Profiler::Instance());
        }))
        .def_static("enable", &Profiler::Enable)
        .def_static("disable", &Profiler::Disable)
        .def_static("dump_and_reset", &Profiler::DumpAndReset)
        .def_property_readonly("is_enabled", &Profiler::IsEnabled)
        // Hard-coded "main" category for trace events
        .def_static(
            "start_trace_event",
            [](const char* name) { TRACE_EVENT_BEGIN("Python", perfetto::DynamicString{name}); })
        .def_static("end_trace_event", [] { TRACE_EVENT_END("Python"); });
}
