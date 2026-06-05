// SPDX-License-Identifier: LGPL-3.0-or-later
#include "SimulationClock.hpp"

#include <cstdint>

SimulationClock::SimulationClock(double dT) : dT(dT)
{
}

void SimulationClock::Advance()
{
    ++iteration;
}

double SimulationClock::ElapsedTime() const
{
    return dT * iteration;
}

uint64_t SimulationClock::Iteration() const
{
    return iteration;
}

double SimulationClock::DT() const
{
    return dT;
}
