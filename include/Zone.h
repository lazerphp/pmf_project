#ifndef ZONE_H
#define ZONE_H

#include <SFML/Graphics.hpp>
#include <random>
#include "Particle.h"
#include "Vector2.h"

class Zone
{
protected:
    sf::FloatRect bounds;
    sf::Color color;

public:
    Zone();
    Zone(const sf::FloatRect &bounds, const sf::Color &color);
    virtual ~Zone();

    bool contains(const Vector2 &position, float margin = 0.0f) const;
    virtual void draw(sf::RenderWindow &window) const;

    const sf::FloatRect &getBounds() const;
};

class SpawnZone : public Zone
{
public:
    SpawnZone();
    SpawnZone(const sf::FloatRect &bounds, const sf::Color &color);

    Vector2 randomPoint(std::mt19937 &rng, std::uniform_real_distribution<float> &dist, float margin = 0.0f) const;
};

class TargetZone : public Zone
{
private:
    int capturedCount;

public:
    TargetZone();
    TargetZone(const sf::FloatRect &bounds, const sf::Color &color);

    bool processParticle(const Particle &particle, float margin = 0.0f);
    int getCapturedCount() const;
};

#endif // ZONE_H
