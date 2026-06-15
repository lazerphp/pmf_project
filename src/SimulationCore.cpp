#include "SimulationCore.h"

#include <stdexcept>

void SimulationCore::validateConfig() const
{
    if (config.particleCount <= 0)
    {
        throw std::invalid_argument("particleCount must be positive");
    }

    if (config.dt <= 0.0f)
    {
        throw std::invalid_argument("dt must be positive");
    }

    if (config.fieldModel.sources.sources.empty())
    {
        throw std::invalid_argument("fieldModel.sources.sources must not be empty");
    }

    for (const auto &source : config.fieldModel.sources.sources)
    {
        if (source.sigma <= 0.0f)
        {
            throw std::invalid_argument("fieldModel.sources.sources[].sigma must be positive");
        }

        if (source.segmentStart.has_value() != source.segmentEnd.has_value())
        {
            throw std::invalid_argument("fieldModel.sources.sources[].segment must contain both start and end");
        }

        if (source.segmentStart.has_value() && source.segmentEnd.has_value())
        {
            const Vector2 delta = *source.segmentEnd - *source.segmentStart;
            const float lengthSquared = delta.x * delta.x + delta.y * delta.y;
            if (lengthSquared == 0.0f)
            {
                throw std::invalid_argument("fieldModel.sources.sources[].segment must have non-zero length");
            }
        }
    }

    if (config.scene.corridor.outline.size() < 3)
    {
        throw std::invalid_argument("scene.corridor.outline must contain at least 3 points");
    }

    for (std::size_t obstacleIndex = 0; obstacleIndex < config.scene.obstacles.size(); ++obstacleIndex)
    {
        if (config.scene.obstacles[obstacleIndex].outline.size() < 3)
        {
            throw std::invalid_argument("scene.obstacles[].outline must contain at least 3 points");
        }
    }

    if (config.scene.spawnZone.width <= 0.0f || config.scene.spawnZone.height <= 0.0f)
    {
        throw std::invalid_argument("scene.spawnZone must have positive width and height");
    }

    if (config.scene.targetZone.width <= 0.0f || config.scene.targetZone.height <= 0.0f)
    {
        throw std::invalid_argument("scene.targetZone must have positive width and height");
    }
}

bool SimulationCore::isInsideAnyObstacle(const Vector2 &position, float margin) const
{
    for (const auto &obstacle : obstacles)
    {
        if (obstacle.contains(position, margin))
        {
            return true;
        }
    }

    return false;
}

bool SimulationCore::isPositionAllowed(const Vector2 &position, float margin) const
{
    return corridor.contains(position, margin) && !isInsideAnyObstacle(position, margin);
}

void SimulationCore::reseed()
{
    if (config.seed.has_value())
    {
        rng.seed(*config.seed);
    }
    else
    {
        rng.seed(std::random_device{}());
    }
}

void SimulationCore::applyExternalForces()
{
    for (auto &particle : particles)
    {
        particle.applyForce(getPotentialForce(particle.position.x, particle.position.y, config.fieldModel));
    }
}

void SimulationCore::applyInteractionForces()
{
    for (std::size_t i = 0; i < particles.size(); ++i)
    {
        for (std::size_t j = i + 1; j < particles.size(); ++j)
        {
            Vector2 force = computePairForce(particles[i].position, particles[j].position);
            particles[i].applyForce(force);
            particles[j].applyForce(force * -1.0f);
        }
    }
}

void SimulationCore::accumulateForces()
{
    for (auto &particle : particles)
    {
        particle.clearForces();
    }

    applyExternalForces();
    applyInteractionForces();
}

SimulationCore::SimulationCore(const RunConfig &runConfig)
    : corridor(runConfig.scene.corridor.outline),
      obstacles(),
      spawnZone(runConfig.scene.spawnZone),
      targetZone(runConfig.scene.targetZone),
      config(runConfig),
      simulationTime(0.0f),
      rng(),
      distPos(0.0f, 1.0f),
      distVel(-Config::Simulation::V_MAX, Config::Simulation::V_MAX)
{
    obstacles.reserve(runConfig.scene.obstacles.size());
    for (const auto &obstacleConfig : runConfig.scene.obstacles)
    {
        obstacles.emplace_back(obstacleConfig.outline);
    }

    validateConfig();
    reseed();
    initParticles();
}

void SimulationCore::initParticles()
{
    particles.clear();
    hasExitedSpawn.clear();
    isCapturedInTarget.clear();
    simulationTime = 0.0f;

    for (int i = 0; i < config.particleCount; ++i)
    {
        Vector2 spawnPoint;
        bool foundValidPosition = false;

        for (int attempt = 0; attempt < Config::Spawn::MAX_ATTEMPTS; ++attempt)
        {
            spawnPoint = spawnZone.randomPoint(rng, distPos, Config::Simulation::PARTICLE_RADIUS);
            foundValidPosition = isPositionAllowed(spawnPoint, Config::Simulation::PARTICLE_RADIUS);

            for (const auto &existingParticle : particles)
            {
                if (!foundValidPosition)
                {
                    break;
                }

                Vector2 delta = spawnPoint - existingParticle.position;
                float distanceSquared = delta.x * delta.x + delta.y * delta.y;

                if (distanceSquared < Config::Spawn::minDistance() * Config::Spawn::minDistance())
                {
                    foundValidPosition = false;
                    break;
                }
            }

            if (foundValidPosition)
            {
                break;
            }
        }

        if (!foundValidPosition)
        {
            break;
        }

        float vx = distVel(rng);
        float vy = distVel(rng);
        particles.emplace_back(spawnPoint.x, spawnPoint.y, vx, vy);
    }

    hasExitedSpawn.resize(particles.size(), false);
    isCapturedInTarget.resize(particles.size(), false);
    stats.reset(particles.size());
}

void SimulationCore::reset()
{
    reseed();
    initParticles();
}

void SimulationCore::step()
{
    accumulateForces();
    const float nextSimulationTime = simulationTime + config.dt;

    for (std::size_t particleIndex = 0; particleIndex < particles.size(); ++particleIndex)
    {
        auto &particle = particles[particleIndex];
        Vector2 previousPosition = particle.position;
        particle.update(config.dt);
        corridor.resolveInnerCollision(particle, previousPosition, Config::Simulation::PARTICLE_RADIUS);
        for (const auto &obstacle : obstacles)
        {
            obstacle.resolveOuterCollision(particle, previousPosition, Config::Simulation::PARTICLE_RADIUS);
        }

        const bool intersectsSpawn = spawnZone.intersects(particle.position, Config::Simulation::PARTICLE_RADIUS);
        if (!hasExitedSpawn[particleIndex] && !intersectsSpawn)
        {
            hasExitedSpawn[particleIndex] = true;
        }

        if (hasExitedSpawn[particleIndex])
        {
            spawnZone.resolveOuterCollision(particle, previousPosition, Config::Simulation::PARTICLE_RADIUS);
        }

        if (!isCapturedInTarget[particleIndex] &&
            targetZone.contains(particle.position, Config::Simulation::PARTICLE_RADIUS))
        {
            isCapturedInTarget[particleIndex] = true;
            stats.recordTargetHit(particleIndex, nextSimulationTime);
        }

        if (isCapturedInTarget[particleIndex])
        {
            targetZone.resolveInnerCollision(particle, previousPosition, Config::Simulation::PARTICLE_RADIUS);
        }
    }

    simulationTime = nextSimulationTime;
}

void SimulationCore::step(int steps)
{
    if (steps <= 0)
    {
        throw std::invalid_argument("steps must be positive");
    }

    for (int stepIndex = 0; stepIndex < steps; ++stepIndex)
    {
        step();
    }
}

void SimulationCore::setFieldModel(const Config::Field::ModelConfig &fieldModel)
{
    config.fieldModel = fieldModel;
}

const std::vector<Particle> &SimulationCore::getParticles() const
{
    return particles;
}

const Corridor &SimulationCore::getCorridor() const
{
    return corridor;
}

const std::vector<Corridor> &SimulationCore::getObstacles() const
{
    return obstacles;
}

const SpawnZone &SimulationCore::getSpawnZone() const
{
    return spawnZone;
}

const TargetZone &SimulationCore::getTargetZone() const
{
    return targetZone;
}

const Config::Field::ModelConfig &SimulationCore::getFieldModel() const
{
    return config.fieldModel;
}

const RunConfig &SimulationCore::getRunConfig() const
{
    return config;
}

const SimulationStats &SimulationCore::getStats() const
{
    return stats;
}

float SimulationCore::getSimulationTime() const
{
    return simulationTime;
}
