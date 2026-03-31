#include "Zone.h"

Zone::Zone() : bounds(0.0f, 0.0f, 0.0f, 0.0f), color(sf::Color::Transparent) {}

Zone::Zone(const sf::FloatRect &bounds_, const sf::Color &color_)
    : bounds(bounds_), color(color_) {}

Zone::~Zone() {}

bool Zone::contains(const Vector2 &position, float margin) const
{
    return position.x >= bounds.left + margin &&
           position.x <= bounds.left + bounds.width - margin &&
           position.y >= bounds.top + margin &&
           position.y <= bounds.top + bounds.height - margin;
}

void Zone::draw(sf::RenderWindow &window) const
{
    sf::RectangleShape shape(sf::Vector2f(bounds.width, bounds.height));
    shape.setPosition(bounds.left, bounds.top);
    shape.setFillColor(color);
    window.draw(shape);
}

const sf::FloatRect &Zone::getBounds() const
{
    return bounds;
}

SpawnZone::SpawnZone() : Zone() {}

SpawnZone::SpawnZone(const sf::FloatRect &bounds_, const sf::Color &color_)
    : Zone(bounds_, color_) {}

Vector2 SpawnZone::randomPoint(std::mt19937 &rng, std::uniform_real_distribution<float> &dist, float margin) const
{
    float x = bounds.left + margin + dist(rng) * (bounds.width - 2.0f * margin);
    float y = bounds.top + margin + dist(rng) * (bounds.height - 2.0f * margin);
    return Vector2(x, y);
}

TargetZone::TargetZone() : Zone(), capturedCount(0) {}

TargetZone::TargetZone(const sf::FloatRect &bounds_, const sf::Color &color_)
    : Zone(bounds_, color_), capturedCount(0) {}

bool TargetZone::processParticle(const Particle &particle, float margin)
{
    if (!contains(particle.position, margin))
    {
        return false;
    }

    ++capturedCount;
    return true;
}

int TargetZone::getCapturedCount() const
{
    return capturedCount;
}
