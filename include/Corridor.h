#ifndef CORRIDOR_H
#define CORRIDOR_H

#include <vector>
#include "Particle.h"

class Corridor
{
private:
    struct WallSegment
    {
        Vector2 start;
        Vector2 end;
    };

    std::vector<Vector2> outline;
    std::vector<WallSegment> wallSegments;

public:
    Corridor();
    explicit Corridor(const std::vector<Vector2> &outline);

    void setOutline(const std::vector<Vector2> &outline);
    bool contains(const Vector2 &position, float margin = 0.0f) const;
    void resolveInnerCollision(Particle &particle, const Vector2 &previousPosition, float margin) const;
    void resolveOuterCollision(Particle &particle, const Vector2 &previousPosition, float margin) const;
    const std::vector<Vector2> &getOutline() const;
};

#endif // CORRIDOR_H
