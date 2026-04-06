#include "Corridor.h"
#include <cmath>
#include <limits>

Corridor::Corridor() : wallColor(sf::Color::White), wallThickness(1.0f) {}

Corridor::Corridor(const std::vector<Vector2> &outline_, const sf::Color &wallColor_, float wallThickness_)
    : wallColor(wallColor_), wallThickness(wallThickness_)
{
    setOutline(outline_);
}

void Corridor::setOutline(const std::vector<Vector2> &outline_)
{
    outline = outline_;
    wallSegments.clear();

    for (std::size_t i = 0; i < outline.size(); ++i)
    {
        const Vector2 &start = outline[i];
        const Vector2 &end = outline[(i + 1) % outline.size()];
        wallSegments.push_back({start, end});
    }
}

bool Corridor::contains(const Vector2 &position, float margin) const
{
    bool inside = false;

    for (std::size_t i = 0, j = outline.size() - 1; i < outline.size(); j = i++)
    {
        const Vector2 &a = outline[i];
        const Vector2 &b = outline[j];

        bool intersects = ((a.y > position.y) != (b.y > position.y)) &&
                          (position.x < (b.x - a.x) * (position.y - a.y) / (b.y - a.y) + a.x);

        if (intersects)
        {
            inside = !inside;
        }
    }

    if (!inside || margin <= 0.0f)
    {
        return inside;
    }

    float minDistanceSquared = std::numeric_limits<float>::max();

    for (const auto &segment : wallSegments)
    {
        Vector2 edge = segment.end - segment.start;
        Vector2 offset = position - segment.start;
        float edgeLengthSquared = edge.x * edge.x + edge.y * edge.y;

        float t = 0.0f;
        if (edgeLengthSquared > 0.0f)
        {
            t = (offset.x * edge.x + offset.y * edge.y) / edgeLengthSquared;
        }

        if (t < 0.0f)
        {
            t = 0.0f;
        }
        else if (t > 1.0f)
        {
            t = 1.0f;
        }

        Vector2 closestPoint = segment.start + edge * t;
        Vector2 delta = position - closestPoint;
        float distanceSquared = delta.x * delta.x + delta.y * delta.y;

        if (distanceSquared < minDistanceSquared)
        {
            minDistanceSquared = distanceSquared;
        }
    }

    return minDistanceSquared >= margin * margin;
}

void Corridor::resolveCollision(Particle &particle, const Vector2 &previousPosition, float margin) const
{
    if (contains(particle.position, margin))
    {
        return;
    }

    Vector2 xOnlyPosition(particle.position.x, previousPosition.y);
    Vector2 yOnlyPosition(previousPosition.x, particle.position.y);

    bool xMovementValid = contains(xOnlyPosition, margin);
    bool yMovementValid = contains(yOnlyPosition, margin);

    if (!xMovementValid)
    {
        particle.position.x = previousPosition.x;
        particle.velocity.x = -particle.velocity.x;
    }

    if (!yMovementValid)
    {
        particle.position.y = previousPosition.y;
        particle.velocity.y = -particle.velocity.y;
    }

    if (xMovementValid && yMovementValid)
    {
        particle.position = previousPosition;
        particle.velocity.x = -particle.velocity.x;
        particle.velocity.y = -particle.velocity.y;
    }

    particle.shape.setPosition({particle.position.x, particle.position.y});
}

void Corridor::draw(sf::RenderWindow &window) const
{
    for (const auto &segment : wallSegments)
    {
        Vector2 delta = segment.end - segment.start;
        float length = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        float angle = std::atan2(delta.y, delta.x) * 180.0f / 3.14159265f;

        sf::RectangleShape wall({length, wallThickness});
        wall.setOrigin({0.0f, wallThickness * 0.5f});
        wall.setPosition({segment.start.x, segment.start.y});
        wall.setRotation(sf::degrees(angle));
        wall.setFillColor(wallColor);
        window.draw(wall);
    }
}
