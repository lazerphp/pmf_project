#ifndef SIMULATION_H
#define SIMULATION_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <random>
#include "Particle.h"
#include "CameraController.h"
#include "Corridor.h"
#include "Zone.h"

// Глобальная функция физического поля (пока заглушка)
Vector2 getPotentialForce(float x, float y);

class Simulation
{
private:
    std::vector<Particle> particles;
    Corridor corridor;
    SpawnZone spawnZone;
    TargetZone targetZone;
    sf::RenderWindow window;
    CameraController cameraController;
    float width, height;

    // Генераторы случайных чисел
    std::mt19937 rng;
    std::uniform_real_distribution<float> distPos;
    std::uniform_real_distribution<float> distVel;

    Vector2 computePairForce(const Particle &a, const Particle &b) const;
    void applyExternalForces();
    void applyInteractionForces();
    void accumulateForces();
    void initParticles();

public:
    Simulation();
    void run();
};

#endif // SIMULATION_H
