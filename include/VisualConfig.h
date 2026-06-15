#ifndef VISUAL_CONFIG_H
#define VISUAL_CONFIG_H

#include <SFML/Graphics/Color.hpp>

namespace VisualConfig
{
namespace Window
{
const int WIDTH = 800;
const int HEIGHT = 600;
const sf::Color BACKGROUND_COLOR(12, 14, 18);
} // namespace Window

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
const sf::Color GRID_MINOR_COLOR(90, 98, 110, 60);
const sf::Color GRID_MAJOR_COLOR(125, 135, 150, 90);
const sf::Color X_AXIS_COLOR(220, 150, 150, 150);
const sf::Color Y_AXIS_COLOR(150, 190, 225, 150);
const sf::Color ORIGIN_COLOR(245, 230, 180, 220);
const float ORIGIN_MARKER_RADIUS = 4.0f;
const float ORIGIN_MARKER_HALF_SIZE = 10.0f;
const sf::Color SPAWN_COLOR(150, 210, 40, 150);
const sf::Color TARGET_COLOR(220, 50, 50, 150);
const sf::Color PARTICLE_COLOR = sf::Color::Cyan;
const sf::Color ATTRACTIVE_SOURCE_COLOR(255, 180, 70);
const sf::Color REPULSIVE_SOURCE_COLOR(80, 190, 255);
const sf::Color SOURCE_CENTER_COLOR(255, 245, 230);
const float SOURCE_INFLUENCE_RADIUS_MULTIPLIER = 3.5f;
} // namespace Visuals
} // namespace VisualConfig

#endif // VISUAL_CONFIG_H
