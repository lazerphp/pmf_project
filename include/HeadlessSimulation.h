#ifndef HEADLESS_SIMULATION_H
#define HEADLESS_SIMULATION_H

#include <cstddef>

#include "RunConfig.h"
#include "SimulationStats.h"

struct HeadlessRunResult
{
    RunConfig runConfig;
    SimulationStats stats;
    float simulatedTime = 0.0f;
    std::size_t stepsExecuted = 0;
};

class HeadlessSimulationRunner
{
public:
    HeadlessRunResult run(const RunConfig &config) const;
};

#endif // HEADLESS_SIMULATION_H
