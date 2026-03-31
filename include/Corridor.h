#ifndef CORRIDOR_H
#define CORRIDOR_H

#include <SFML/Graphics.hpp>
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
    sf::Color wallColor;
    float wallThickness;

public:
    Corridor();
    Corridor(const std::vector<Vector2> &outline, const sf::Color &wallColor, float wallThickness);

    void setOutline(const std::vector<Vector2> &outline);
    bool contains(const Vector2 &position, float margin = 0.0f) const;
    void resolveCollision(Particle &particle, const Vector2 &previousPosition, float margin) const;
    void draw(sf::RenderWindow &window) const;
};

#endif // CORRIDOR_H
