#include "Particle.h"
#include "Config.h"

Vector2::Vector2(float _x, float _y) : x(_x), y(_y) {}
Vector2 Vector2::operator+(const Vector2 &o) const { return Vector2(x + o.x, y + o.y); }
Vector2 Vector2::operator-(const Vector2 &o) const { return Vector2(x - o.x, y - o.y); }
Vector2 Vector2::operator*(float s) const { return Vector2(x * s, y * s); }
void Vector2::operator+=(const Vector2 &o)
{
    x += o.x;
    y += o.y;
}

Particle::Particle(float x, float y, float vx, float vy)
    : position(x, y), velocity(vx, vy), acceleration(0, 0)
{

    shape.setRadius(Config::Simulation::PARTICLE_RADIUS);
    shape.setFillColor(sf::Color::Cyan);
    shape.setOrigin(Config::Simulation::PARTICLE_RADIUS, Config::Simulation::PARTICLE_RADIUS);
    shape.setPosition(position.x, position.y);
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
    shape.setPosition(position.x, position.y);
}

void Particle::applyForce(const Vector2 &force)
{
    acceleration += force;
}
