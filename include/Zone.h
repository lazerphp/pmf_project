#ifndef ZONE_H
#define ZONE_H

#include <random>

#include "Particle.h"
#include "Rect.h"
#include "Vector2.h"

class Zone
{
protected:
    Rect bounds;

public:
    Zone();
    explicit Zone(const Rect &bounds);
    virtual ~Zone();

    bool contains(const Vector2 &position, float margin = 0.0f) const;
    bool intersects(const Vector2 &position, float margin = 0.0f) const;
    void resolveInnerCollision(Particle &particle, const Vector2 &previousPosition, float margin = 0.0f) const;
    void resolveOuterCollision(Particle &particle, const Vector2 &previousPosition, float margin = 0.0f) const;
    const Rect &getBounds() const;
};

class SpawnZone : public Zone
{
public:
    SpawnZone();
    explicit SpawnZone(const Rect &bounds);

    Vector2 randomPoint(std::mt19937 &rng, std::uniform_real_distribution<float> &dist, float margin = 0.0f) const;
};

class TargetZone : public Zone
{
public:
    TargetZone();
    explicit TargetZone(const Rect &bounds);
};

#endif // ZONE_H
