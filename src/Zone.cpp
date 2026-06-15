#include "Zone.h"

namespace
{
    float clampFloat(float value, float minValue, float maxValue)
    {
        if (value < minValue)
        {
            return minValue;
        }

        if (value > maxValue)
        {
            return maxValue;
        }

        return value;
    }

    bool intersectsZone(const Zone &zone, const Vector2 &position, float margin)
    {
        const Rect &bounds = zone.getBounds();
        const float closestX = clampFloat(position.x, bounds.x, bounds.x + bounds.width);
        const float closestY = clampFloat(position.y, bounds.y, bounds.y + bounds.height);
        const float dx = position.x - closestX;
        const float dy = position.y - closestY;
        return dx * dx + dy * dy < margin * margin;
    }

    void resolveCollisionAgainstZone(const Zone &zone, Particle &particle, const Vector2 &previousPosition, float margin, bool shouldStayInside)
    {
        const bool positionValid = shouldStayInside ? zone.contains(particle.position, margin)
                                                    : !intersectsZone(zone, particle.position, margin);
        if (positionValid)
        {
            return;
        }

        Vector2 xOnlyPosition(particle.position.x, previousPosition.y);
        Vector2 yOnlyPosition(previousPosition.x, particle.position.y);

        bool xMovementValid = shouldStayInside ? zone.contains(xOnlyPosition, margin)
                                               : !intersectsZone(zone, xOnlyPosition, margin);
        bool yMovementValid = shouldStayInside ? zone.contains(yOnlyPosition, margin)
                                               : !intersectsZone(zone, yOnlyPosition, margin);

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
    }
} // namespace

Zone::Zone() : bounds{0.0f, 0.0f, 0.0f, 0.0f} {}

Zone::Zone(const Rect &bounds_)
    : bounds(bounds_) {}

Zone::~Zone() {}

bool Zone::contains(const Vector2 &position, float margin) const
{
    return position.x >= bounds.x + margin &&
           position.x <= bounds.x + bounds.width - margin &&
           position.y >= bounds.y + margin &&
           position.y <= bounds.y + bounds.height - margin;
}

bool Zone::intersects(const Vector2 &position, float margin) const
{
    return intersectsZone(*this, position, margin);
}

void Zone::resolveInnerCollision(Particle &particle, const Vector2 &previousPosition, float margin) const
{
    resolveCollisionAgainstZone(*this, particle, previousPosition, margin, true);
}

void Zone::resolveOuterCollision(Particle &particle, const Vector2 &previousPosition, float margin) const
{
    resolveCollisionAgainstZone(*this, particle, previousPosition, margin, false);
}

const Rect &Zone::getBounds() const
{
    return bounds;
}

SpawnZone::SpawnZone() : Zone() {}

SpawnZone::SpawnZone(const Rect &bounds_)
    : Zone(bounds_) {}

Vector2 SpawnZone::randomPoint(std::mt19937 &rng, std::uniform_real_distribution<float> &dist, float margin) const
{
    float x = bounds.x + margin + dist(rng) * (bounds.width - 2.0f * margin);
    float y = bounds.y + margin + dist(rng) * (bounds.height - 2.0f * margin);
    return Vector2(x, y);
}

TargetZone::TargetZone() : Zone() {}

TargetZone::TargetZone(const Rect &bounds_)
    : Zone(bounds_) {}
