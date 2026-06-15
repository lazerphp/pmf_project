#ifndef PARTICLE_H
#define PARTICLE_H

#include "Vector2.h"

class Particle
{
public:
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;

    Particle(float x, float y, float vx, float vy);

    void clearForces();
    void update(float dt);
    void applyForce(const Vector2 &force);
};

#endif // PARTICLE_H
