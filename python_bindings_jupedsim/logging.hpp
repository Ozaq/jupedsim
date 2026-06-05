// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Logger.hpp"

class LogCallbackOwner
{
public:
    using LogCallback = Logging::Logger::LogCallback;

    LogCallback Debug{};
    LogCallback Info{};
    LogCallback Warning{};
    LogCallback Error{};

public:
    static LogCallbackOwner& Instance();
};
