#ifndef CONFIG_H
#define CONFIG_H

#include <optional>
#include <vector>

#include "Rect.h"
#include "Vector2.h"

namespace Config
{
namespace Simulation
{
const int PARTICLE_COUNT = 50;
const int PHYSICS_STEPS_PER_FRAME = 5;
const float DT = 0.001f;
const float PARTICLE_RADIUS = 5.0f;
const float V_MAX = 200.0f;
} // namespace Simulation

namespace World
{
// Начало координат находится в левом верхнем углу сцены.
// Ось X направлена вправо, ось Y направлена вниз, как в системе координат SFML.
const float GRID_UNIT = 20.0f;

inline float grid(float value)
{
    return value * GRID_UNIT;
}

inline float gridX(float x)
{
    return grid(x);
}

inline float gridY(float y)
{
    return grid(y);
}

inline Vector2 gridPoint(float x, float y)
{
    return Vector2(gridX(x), gridY(y));
}

inline Rect gridRect(float x, float y, float width, float height)
{
    return Rect{gridX(x), gridY(y), grid(width), grid(height)};
}
} // namespace World

namespace LennardJones
{
const float EPSILON = 900.0f;
const float CONTACT_DISTANCE_TO_SIGMA = 1.122462048f;
const float CUTOFF_MULTIPLIER = 2.5f;
const float MIN_DISTANCE_MULTIPLIER = 0.6f;

inline float sigma()
{
    return (2.0f * Simulation::PARTICLE_RADIUS) / CONTACT_DISTANCE_TO_SIGMA;
}

inline float cutoff()
{
    return CUTOFF_MULTIPLIER * sigma();
}

inline float minPairDistance()
{
    return MIN_DISTANCE_MULTIPLIER * sigma();
}
} // namespace LennardJones

namespace Spawn
{
const int MAX_ATTEMPTS = 200;
const float MIN_DISTANCE_MULTIPLIER = 2.0f;

inline float minDistance()
{
    return MIN_DISTANCE_MULTIPLIER * Simulation::PARTICLE_RADIUS;
}
} // namespace Spawn

namespace Field
{
struct Source
{
    Vector2 center;
    float strength;
    float sigma;
    std::optional<Vector2> segmentStart;
    std::optional<Vector2> segmentEnd;
};

struct SourcesParams
{
    std::vector<Source> sources;
};

struct ModelConfig
{
    SourcesParams sources;
};
} // namespace Field
} // namespace Config

#endif // CONFIG_H
