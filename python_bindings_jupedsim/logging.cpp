// SPDX-License-Identifier: LGPL-3.0-or-later
#include "logging.hpp"

#include "Logger.hpp"

#include <pybind11/functional.h> // IWYU pragma: keep
#include <pybind11/pybind11.h>

#include <string>

namespace py = pybind11;

// TODO(kkratz): I think this can now be replaced by lifetime annotations, i.e. py::keep_alive...
LogCallbackOwner& LogCallbackOwner::Instance()
{
    static LogCallbackOwner instance;
    return instance;
}

void init_logging(py::module_& m)
{
    auto atexit = py::module_::import("atexit");
    atexit.attr("register")(py::cpp_function([]() {
        auto& owner = LogCallbackOwner::Instance();
        owner.Debug = {};
        owner.Info = {};
        owner.Warning = {};
        owner.Error = {};
    }));
    m.def("set_debug_callback", [](LogCallbackOwner::LogCallback callback) {
        LogCallbackOwner::Instance().Debug = callback;
        Logging::Logger::Instance().SetDebugCallback(
            [](const std::string& msg) { LogCallbackOwner::Instance().Debug(msg); });
    });
    m.def("set_info_callback", [](LogCallbackOwner::LogCallback callback) {
        LogCallbackOwner::Instance().Info = callback;
        Logging::Logger::Instance().SetInfoCallback(
            [](const std::string& msg) { LogCallbackOwner::Instance().Info(msg); });
    });
    m.def("set_warning_callback", [](LogCallbackOwner::LogCallback callback) {
        LogCallbackOwner::Instance().Warning = callback;
        Logging::Logger::Instance().SetWarningCallback(
            [](const std::string& msg) { LogCallbackOwner::Instance().Warning(msg); });
    });
    m.def("set_error_callback", [](LogCallbackOwner::LogCallback callback) {
        LogCallbackOwner::Instance().Error = callback;
        Logging::Logger::Instance().SetErrorCallback(
            [](const std::string& msg) { LogCallbackOwner::Instance().Error(msg); });
    });
}
