#ifndef CONFIG_H
#define CONFIG_H

#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/Rect.hpp>
#include "Vector2.h"

namespace Config
{
namespace Window
{
const int WIDTH = 800;
const int HEIGHT = 600;
const sf::Color BACKGROUND_COLOR(12, 14, 18);
} // namespace Window

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

inline sf::FloatRect gridRect(float x, float y, float width, float height)
{
    return sf::FloatRect({gridX(x), gridY(y)}, {grid(width), grid(height)});
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
// Постоянная сила вправо в стартовом участке коридора.
// const float START_DRIVE_X = 18.0f;
const float START_DRIVE_X = 40.0f;
// Постоянная сила вправо в верхнем горизонтальном участке.
// const float TOP_DRIVE_X = 18.0f;
const float TOP_DRIVE_X = 25.0f;
// Насколько сильно частицы прижимаются к центральной линии верхнего участка.
// const float TOP_CENTERING_Y = 0.12f;
const float TOP_CENTERING_Y = 0.2f;
// Постоянная сила вниз в правом вертикальном участке.
const float RIGHT_DRIVE_Y = 18.0f;
// Насколько сильно частицы прижимаются к центральной линии правого участка.
const float RIGHT_CENTERING_X = 0.12f;
// Постоянная сила вниз в нижнем вертикальном/переходном участке.
const float BOTTOM_DRIVE_Y = 18.0f;
// Насколько сильно частицы прижимаются к центральной линии нижнего участка.
const float BOTTOM_CENTERING_X = 0.12f;
// Локальное притяжение к центру цели по оси X в финальном участке.
const float TARGET_ATTRACTION_X = 0.035f;
// Локальное притяжение к центру цели по оси Y в финальном участке.
const float TARGET_ATTRACTION_Y = 0.035f;
// Слабое глобальное притяжение к цели, действующее по всему коридору.
const float GLOBAL_TARGET_ATTRACTION = 0.035f;
// Ограничение на модуль силы внешнего поля.
const float MAX_FORCE = 160.0f;

// До этой X-координаты действует стартовый участок поля.
const float TOP_SEGMENT_X_MAX = World::gridX(9.0f);
// Выше этой Y-координаты частица считается в верхнем горизонтальном участке.
const float TOP_SEGMENT_Y_MAX = World::gridY(8.5f);
// Правее этой X-координаты начинается правый вертикальный участок.
const float RIGHT_SEGMENT_X_MIN = World::gridX(21.5f);
// До этой Y-координаты действует правый вертикальный участок.
const float RIGHT_SEGMENT_Y_MAX = World::gridY(18.1f);
// До этой Y-координаты действует нижний участок перед зоной цели.
const float LOWER_SEGMENT_Y_MAX = World::gridY(22.5f);

// Y-координата центральной линии верхнего участка.
const float UPPER_LANE_CENTER_Y = World::gridY(6.5f);
// X-координата центральной линии правого вертикального участка.
const float RIGHT_LANE_CENTER_X = World::gridX(26.5f);
// X-координата центральной линии нижнего участка.
const float LOWER_LANE_CENTER_X = World::gridX(16.5f);
// Центр, к которому поле притягивает частицы в конце и глобально.
const Vector2 TARGET_CENTER = World::gridPoint(16.5f, 24.75f);

struct Params
{
    float startDriveX;
    float topDriveX;
    float topCenteringY;
    float rightDriveY;
    float rightCenteringX;
    float bottomDriveY;
    float bottomCenteringX;
    float targetAttractionX;
    float targetAttractionY;
    float globalTargetAttraction;
    float maxForce;
};

inline Params defaultParams()
{
    return Params{
        START_DRIVE_X,
        TOP_DRIVE_X,
        TOP_CENTERING_Y,
        RIGHT_DRIVE_Y,
        RIGHT_CENTERING_X,
        BOTTOM_DRIVE_Y,
        BOTTOM_CENTERING_X,
        TARGET_ATTRACTION_X,
        TARGET_ATTRACTION_Y,
        GLOBAL_TARGET_ATTRACTION,
        MAX_FORCE,
    };
}
} // namespace Field

namespace Camera
{
const float ZOOM_STEP = 1.1f;
const float MIN_ZOOM = 0.35f;
const float MAX_ZOOM = 4.0f;
} // namespace Camera

namespace Visuals
{
const sf::Color WALL_COLOR(255, 255, 255);
const float WALL_THICKNESS = 4.0f;
const sf::Color SPAWN_COLOR(150, 210, 40, 150);
const sf::Color TARGET_COLOR(220, 50, 50, 150);
} // namespace Visuals
} // namespace Config

#endif // CONFIG_H
