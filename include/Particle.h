#ifndef PARTICLE_H
#define PARTICLE_H

#include <SFML/Graphics.hpp>
#include "Vector2.h"

class Particle
{
public:
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;
    sf::CircleShape shape;

    Particle(float x, float y, float vx, float vy);

    void update(float dt);
    void applyForce(const Vector2 &force);
};

#endif // PARTICLE_H