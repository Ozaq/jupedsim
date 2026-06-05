#include "Tracing.hpp"

#include "Logger.hpp"

#include <perfetto.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

PERFETTO_TRACK_EVENT_STATIC_STORAGE();

namespace
{
perfetto::TraceConfig buildDefaultTraceConfig()
{
    perfetto::TraceConfig cfg;

    auto* buffer = cfg.add_buffers();
    buffer->set_size_kb(8192);

    auto* ds_cfg = cfg.add_data_sources()->mutable_config();
    ds_cfg->set_name("track_event");

    return cfg;
}
} // namespace

void Profiler::createSession()
{
    if(tracingSession) {
        return;
    }

    if(!perfetto::Tracing::IsInitialized()) {
        perfetto::TracingInitArgs args;
        args.backends |= perfetto::kInProcessBackend;
        perfetto::Tracing::Initialize(args);
    }

    perfetto::TrackEvent::Register();

    tracingSession = perfetto::Tracing::NewTrace();
    tracingSession->Setup(buildDefaultTraceConfig());
    tracingSession->StartBlocking();
}

void Profiler::writeAndResetSession(const std::string& filename)
{
    if(!tracingSession) {
        return;
    }

    perfetto::TrackEvent::Flush();
    tracingSession->StopBlocking();
    const auto trace_data = tracingSession->ReadTraceBlocking();
    if(filename.empty()) {
        tracingSession.reset();
        return;
    }
    std::ofstream output(filename, std::ios::binary | std::ios::trunc);
    if(output.is_open()) {
        output.write(trace_data.data(), static_cast<std::streamsize>(trace_data.size()));
        output.flush();
    } else {
        LOG_ERROR("Failed to open Perfetto output file: {}", filename);
    }

    tracingSession.reset();
}

void Profiler::Enable()
{
    auto& instance = Profiler::Instance();
    if(instance.enabled) {
        return;
    }

    instance.createSession();
    instance.enabled = true;
}

void Profiler::Disable()
{
    auto& instance = Profiler::Instance();
    if(!instance.enabled && !instance.tracingSession) {
        return;
    }

    instance.writeAndResetSession("");
    instance.enabled = false;
}

void Profiler::DumpAndReset(const std::string& filename)
{
    auto& instance = Profiler::Instance();
    instance.writeAndResetSession(filename);
    instance.enabled = false;
}

Profiler Profiler::profiler{};