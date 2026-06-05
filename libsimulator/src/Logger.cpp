// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Logger.hpp"

#include <string>

namespace Logging
{

Logger& Logger::Instance()
{
    static Logger logger;
    return logger;
}

void Logger::SetDebugCallback(LogCallback&& cb)
{
    debugMsgCb = cb;
}

void Logger::ClearDebugCallback()
{
    debugMsgCb = {};
}

void Logger::LogDebugMessage(const std::string& msg)
{
    if(debugMsgCb) {
        debugMsgCb(msg);
    }
}

void Logger::SetInfoCallback(LogCallback&& cb)
{
    infoMsgCb = cb;
}

void Logger::ClearInfoCallback()
{
    infoMsgCb = {};
}

void Logger::LogInfoMessage(const std::string& msg)
{
    if(infoMsgCb) {
        infoMsgCb(msg);
    }
}

void Logger::SetWarningCallback(LogCallback&& cb)
{
    warningMsgCb = cb;
}

void Logger::ClearWarningCallback()
{
    warningMsgCb = {};
}

void Logger::LogWarningMessage(const std::string& msg)
{
    if(warningMsgCb) {
        warningMsgCb(msg);
    }
}

void Logger::SetErrorCallback(LogCallback&& cb)
{
    errorMsgCb = cb;
}

void Logger::ClearErrorCallback()
{
    errorMsgCb = {};
}

void Logger::LogErrorMessage(const std::string& msg)
{
    if(errorMsgCb) {
        errorMsgCb(msg);
    }
}

void Logger::ClearAllCallbacks()
{
    ClearDebugCallback();
    ClearInfoCallback();
    ClearWarningCallback();
    ClearErrorCallback();
}

} // namespace Logging
