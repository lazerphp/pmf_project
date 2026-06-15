#ifndef SIMULATION_STATS_H
#define SIMULATION_STATS_H

#include <cstddef>
#include <optional>
#include <vector>

struct SimulationStats
{
    std::vector<bool> reachedTarget;
    std::vector<std::optional<float>> firstHitTime;
    std::size_t uniqueTargetHits = 0;
    std::optional<float> t80;

    void reset(std::size_t particleCount);
    void recordTargetHit(std::size_t particleIndex, float hitTime);
    std::size_t targetThresholdCount() const;
    std::size_t missingTargetHits() const;
    float missingTargetFraction() const;
    float computeQuality(float maxSimulationTime, float penaltyWeight = 1.0f) const;
};

#endif // SIMULATION_STATS_H
