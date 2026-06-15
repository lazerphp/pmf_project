#include "Particle.h"

Particle::Particle(float x, float y, float vx, float vy)
    : position(x, y), velocity(vx, vy), acceleration(0, 0)
{
}

void Particle::clearForces()
{
    acceleration = Vector2(0, 0);
}

void Particle::update(float dt)
{
    velocity += acceleration * dt;
    position += velocity * dt;
    clearForces();
}

void Particle::applyForce(const Vector2 &force)
{
    acceleration += force;
}
