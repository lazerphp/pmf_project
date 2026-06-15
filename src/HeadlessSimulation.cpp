#include "HeadlessSimulation.h"

#include <stdexcept>

#include "SimulationCore.h"

HeadlessRunResult HeadlessSimulationRunner::run(const RunConfig &config) const
{
    if (config.maxSimulationTime <= 0.0f)
    {
        throw std::invalid_argument("maxSimulationTime must be positive");
    }

    SimulationCore core(config);

    std::size_t stepsExecuted = 0;
    while (core.getSimulationTime() < config.maxSimulationTime && !core.getStats().t80.has_value())
    {
        core.step();
        ++stepsExecuted;
    }

    return HeadlessRunResult{
        config,
        core.getStats(),
        core.getSimulationTime(),
        stepsExecuted,
    };
}
