#include "SimulationStats.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

void SimulationStats::reset(std::size_t particleCount)
{
    reachedTarget.assign(particleCount, false);
    firstHitTime.assign(particleCount, std::nullopt);
    uniqueTargetHits = 0;
    t80.reset();
}

void SimulationStats::recordTargetHit(std::size_t particleIndex, float hitTime)
{
    if (particleIndex >= reachedTarget.size() || reachedTarget[particleIndex])
    {
        return;
    }

    reachedTarget[particleIndex] = true;
    firstHitTime[particleIndex] = hitTime;
    ++uniqueTargetHits;

    const std::size_t threshold = targetThresholdCount();
    if (threshold > 0 && uniqueTargetHits >= threshold && !t80.has_value())
    {
        t80 = hitTime;
    }
}

std::size_t SimulationStats::targetThresholdCount() const
{
    if (reachedTarget.empty())
    {
        return 0;
    }

    return std::max<std::size_t>(
        1,
        static_cast<std::size_t>(std::ceil(static_cast<double>(reachedTarget.size()) * 0.8)));
}

std::size_t SimulationStats::missingTargetHits() const
{
    const std::size_t threshold = targetThresholdCount();
    const std::size_t clampedHits = std::min(uniqueTargetHits, threshold);
    return threshold - clampedHits;
}

float SimulationStats::missingTargetFraction() const
{
    const std::size_t threshold = targetThresholdCount();
    if (threshold == 0)
    {
        return 0.0f;
    }

    return static_cast<float>(missingTargetHits()) / static_cast<float>(threshold);
}

float SimulationStats::computeQuality(float maxSimulationTime, float penaltyWeight) const
{
    if (maxSimulationTime <= 0.0f)
    {
        throw std::invalid_argument("maxSimulationTime must be positive");
    }

    if (penaltyWeight < 0.0f)
    {
        throw std::invalid_argument("penaltyWeight must be non-negative");
    }

    if (t80.has_value())
    {
        return *t80;
    }

    return maxSimulationTime + penaltyWeight * missingTargetFraction();
}
