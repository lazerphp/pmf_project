#include "Simulation.h"
#include <iostream>

// Настройки симуляции
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const int PARTICLE_COUNT = 1000;
const float V_MAX = 200.0f;
const float DT = 0.01f;
const float PARTICLE_RADIUS = 5.0f;

Vector2 getPotentialForce(float x, float y)
{
    return Vector2(0, 0);
}

Simulation::Simulation()
    : window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Modeling: Ideal Gas"),
      width(WINDOW_WIDTH), height(WINDOW_HEIGHT),
      rng(std::random_device{}()),
      distPos(0, 1),
      distVel(-V_MAX, V_MAX)
{
    window.setFramerateLimit(60);
    // window.setView(window.getDefaultView()); // Чтобы частицы не плющило

    initParticles();
}

void Simulation::initParticles()
{
    particles.clear();
    for (int i = 0; i < PARTICLE_COUNT; ++i)
    {
        float x = distPos(rng) * (width - 2 * PARTICLE_RADIUS) + PARTICLE_RADIUS;
        float y = distPos(rng) * (height - 2 * PARTICLE_RADIUS) + PARTICLE_RADIUS;
        float vx = distVel(rng);
        float vy = distVel(rng);
        particles.emplace_back(x, y, vx, vy);
    }
}

void Simulation::handleWallCollisions(Particle &p)
{
    if (p.position.x < PARTICLE_RADIUS)
    {
        p.position.x = PARTICLE_RADIUS + (PARTICLE_RADIUS - p.position.x);
        p.velocity.x = -p.velocity.x;
    }
    else if (p.position.x > width - PARTICLE_RADIUS)
    {
        p.position.x = (width - PARTICLE_RADIUS) - (p.position.x - (width - PARTICLE_RADIUS));
        p.velocity.x = -p.velocity.x;
    }

    if (p.position.y < PARTICLE_RADIUS)
    {
        p.position.y = PARTICLE_RADIUS + (PARTICLE_RADIUS - p.position.y);
        p.velocity.y = -p.velocity.y;
    }
    else if (p.position.y > height - PARTICLE_RADIUS)
    {
        p.position.y = (height - PARTICLE_RADIUS) - (p.position.y - (height - PARTICLE_RADIUS));
        p.velocity.y = -p.velocity.y;
    }
}

void Simulation::run()
{
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space)
            {
                initParticles();
            }

            // if (event.type == sf::Event::Resized) {
            //     window.setView(window.getDefaultView());
            // }
        }

        for (auto &p : particles)
        {
            Vector2 force = getPotentialForce(p.position.x, p.position.y);
            p.applyForce(force);
            p.update(DT);
            handleWallCollisions(p);
        }

        window.clear(sf::Color::Black);
        // window.setView(window.getDefaultView());

        // Внутри Simulation::run(), после pollEvent, перед отрисовкой

        // Получаем текущий реальный размер окна (который мог измениться)
        unsigned int winWidth = window.getSize().x;
        unsigned int winHeight = window.getSize().y;

        // Наши фиксированные логические размеры
        float logicalWidth = static_cast<float>(WINDOW_WIDTH);
        float logicalHeight = static_cast<float>(WINDOW_HEIGHT);

        // Считаем соотношения сторон
        float windowRatio = static_cast<float>(winWidth) / static_cast<float>(winHeight);
        float logicalRatio = logicalWidth / logicalHeight;

        sf::View view = window.getDefaultView();
        sf::FloatRect viewport;

        if (windowRatio > logicalRatio)
        {
            // Окно шире, чем нужно -> черные полосы слева и справа
            float newWidth = logicalRatio * winHeight;
            viewport.left = (winWidth - newWidth) / 2.0f / winWidth;
            viewport.top = 0.0f;
            viewport.width = newWidth / winWidth;
            viewport.height = 1.0f;
        }
        else
        {
            // Окно выше, чем нужно -> черные полосы сверху и снизу
            float newHeight = winWidth / logicalRatio;
            viewport.left = 0.0f;
            viewport.top = (winHeight - newHeight) / 2.0f / winHeight;
            viewport.width = 1.0f;
            viewport.height = newHeight / winHeight;
        }

        view.setViewport(viewport);
        window.setView(view);

        sf::RectangleShape border(sf::Vector2f(width, height));
        border.setFillColor(sf::Color::Transparent);
        border.setOutlineColor(sf::Color::White);
        border.setOutlineThickness(2);
        window.draw(border);

        for (const auto &p : particles)
            window.draw(p.shape);

        window.display();
    }
}