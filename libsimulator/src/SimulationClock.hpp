// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>

class SimulationClock
{
    uint64_t iteration{0};
    double dT;

public:
    explicit SimulationClock(double dT);

    void Advance();

    double ElapsedTime() const;

    uint64_t Iteration() const;

    double DT() const;
};
