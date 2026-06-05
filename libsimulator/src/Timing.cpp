// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Timing.hpp"

#include <fmt/core.h>
#include <perfetto.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <utility>

namespace cr = std::chrono;

TimerEntry::TimerEntry(TimerEntry&& other) noexcept
    : startedAt(std::move(other.startedAt))
    , durationInMicroseconds(other.durationInMicroseconds)
    , running(other.running)
{
    other.durationInMicroseconds = 0;
    other.running = false;
}

TimerEntry& TimerEntry::operator=(TimerEntry&& other) noexcept
{
    if(this != &other) {
        startedAt = std::move(other.startedAt);
        durationInMicroseconds = other.durationInMicroseconds;
        running = other.running;
        other.durationInMicroseconds = 0;
        other.running = false;
    }
    return *this;
}

void TimerEntry::Start()
{
    if(!running) {
        running = true;
        startedAt = cr::high_resolution_clock::now();
    }
}

void TimerEntry::Stop()
{
    if(running) {
        running = false;
        durationInMicroseconds += std::chrono::duration_cast<std::chrono::microseconds>(
                                        std::chrono::high_resolution_clock::now() - startedAt)
                                        .count();
    }
}

uint64_t TimerEntry::GetDurationInMicroseconds() const
{
    if(running) {
        return durationInMicroseconds +
               std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::high_resolution_clock::now() - startedAt)
                   .count();
    }
    return durationInMicroseconds;
}

void Timer::PushTimerProbe(std::string_view name, int timer_probe_level)
{
    if(timer_probe_level > maxLogLevel) {
        return;
    }
    std::string name_str(name);
    auto iter = timerMap.find(name_str);
    // use emplace to avoid multiple lookups and unnecessary default construction of TimerEntry
    if(iter == timerMap.end()) {
        timerMap.emplace(name_str, TimerEntry()).first->second.Start();
    } else {
        iter->second.Start();
    }
}

void Timer::PopTimerProbe(const std::string_view name)
{
    // use auto iter = timer_map.find(name) to avoid multiple lookups
    auto iter = timerMap.find(std::string(name));
    if(iter != timerMap.end()) {
        iter->second.Stop();
    }
}

TimerEntry::duration_type Timer::GetDuration(const std::string_view name) const
{
    auto iter = timerMap.find(std::string(name));
    if(iter != timerMap.end()) {
        return iter->second.GetDurationInMicroseconds();
    }
    return 0;
}

std::map<std::string, TimerEntry::duration_type> Timer::GetDurations() const
{
    std::map<std::string, TimerEntry::duration_type> entries;
    for(const auto& [name, trace] : timerMap) {
        entries.emplace(name, trace.GetDurationInMicroseconds());
    }
    return entries;
}