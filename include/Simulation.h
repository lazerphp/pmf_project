#ifndef SIMULATION_H
#define SIMULATION_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <random>
#include "Particle.h"

// Глобальная функция физического поля (пока заглушка)
Vector2 getPotentialForce(float x, float y);

class Simulation
{
private:
    std::vector<Particle> particles;
    sf::RenderWindow window;
    float width, height;

    // Генераторы случайных чисел
    std::mt19937 rng;
    std::uniform_real_distribution<float> distPos;
    std::uniform_real_distribution<float> distVel;

    void initParticles();
    void handleWallCollisions(Particle &p);

public:
    Simulation();
    void run();
};

#endif // SIMULATION_H