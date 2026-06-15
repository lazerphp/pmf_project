#ifndef RUN_CONFIG_H
#define RUN_CONFIG_H

#include <optional>
#include <string>
#include <vector>

#include "Config.h"

struct QualityConfig
{
    std::string mode = "t80_or_penalized_missing_fraction";
    float penaltyWeight = 1.0f;
};

struct PolygonConfig
{
    std::vector<Vector2> outline;
};

struct SceneConfig
{
    PolygonConfig corridor;
    std::vector<PolygonConfig> obstacles;
    Rect spawnZone{0.0f, 0.0f, 0.0f, 0.0f};
    Rect targetZone{0.0f, 0.0f, 0.0f, 0.0f};
};

struct RunConfig
{
    int particleCount = Config::Simulation::PARTICLE_COUNT;
    int physicsStepsPerFrame = Config::Simulation::PHYSICS_STEPS_PER_FRAME;
    float dt = Config::Simulation::DT;
    float maxSimulationTime = 10.0f;
    std::optional<unsigned int> seed;
    Config::Field::ModelConfig fieldModel;
    SceneConfig scene;
    QualityConfig quality;
};

#endif // RUN_CONFIG_H
