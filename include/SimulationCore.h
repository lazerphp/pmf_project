#ifndef SIMULATION_CORE_H
#define SIMULATION_CORE_H

#include <random>
#include <vector>

#include "Corridor.h"
#include "Particle.h"
#include "PhysicsForces.h"
#include "RunConfig.h"
#include "SimulationStats.h"
#include "Zone.h"

class SimulationCore
{
private:
    std::vector<Particle> particles;
    std::vector<bool> hasExitedSpawn;
    std::vector<bool> isCapturedInTarget;
    Corridor corridor;
    std::vector<Corridor> obstacles;
    SpawnZone spawnZone;
    TargetZone targetZone;
    RunConfig config;
    SimulationStats stats;
    float simulationTime;

    std::mt19937 rng;
    std::uniform_real_distribution<float> distPos;
    std::uniform_real_distribution<float> distVel;

    void validateConfig() const;
    void reseed();
    void applyExternalForces();
    void applyInteractionForces();
    void accumulateForces();
    void initParticles();
    bool isPositionAllowed(const Vector2 &position, float margin) const;
    bool isInsideAnyObstacle(const Vector2 &position, float margin) const;

public:
    explicit SimulationCore(const RunConfig &runConfig);

    void reset();
    void step();
    void step(int steps);
    void setFieldModel(const Config::Field::ModelConfig &fieldModel);

    const std::vector<Particle> &getParticles() const;
    const Corridor &getCorridor() const;
    const std::vector<Corridor> &getObstacles() const;
    const SpawnZone &getSpawnZone() const;
    const TargetZone &getTargetZone() const;
    const Config::Field::ModelConfig &getFieldModel() const;
    const RunConfig &getRunConfig() const;
    const SimulationStats &getStats() const;
    float getSimulationTime() const;
};

#endif // SIMULATION_CORE_H
