#include "Simulation.h"

// Настройки симуляции
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const int PARTICLE_COUNT = 50;
const float V_MAX = 200.0f;
const float DT = 0.01f;
const float PARTICLE_RADIUS = 5.0f;
const float MORSE_DEPTH = 1400.0f;
const float MORSE_WIDTH = 0.09f;
const float MORSE_EQUILIBRIUM_DISTANCE = 18.0f;
const float PAIR_FORCE_CUTOFF = 90.0f;
const float MIN_PAIR_DISTANCE = 2.0f;
const float FIELD_DRIVE_STRENGTH = 18.0f;
const float FIELD_CENTERING_STRENGTH = 0.12f;
const float FIELD_TARGET_STRENGTH = 0.035f;
const float FIELD_MAX_FORCE = 160.0f;
const float CAMERA_ZOOM_STEP = 1.1f;
const float CAMERA_MIN_ZOOM = 0.35f;
const float CAMERA_MAX_ZOOM = 4.0f;
const sf::Color BACKGROUND_COLOR(12, 14, 18);
const sf::Color WALL_COLOR(255, 255, 255);
const float WALL_THICKNESS = 4.0f;
const sf::Color SPAWN_COLOR(150, 210, 40, 150);
const sf::Color TARGET_COLOR(220, 50, 50, 150);

namespace
{
float clampMagnitude(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

Vector2 limitForce(const Vector2 &force, float maxMagnitude)
{
    float magnitudeSquared = force.x * force.x + force.y * force.y;

    if (magnitudeSquared == 0.0f)
    {
        return force;
    }

    float magnitude = std::sqrt(magnitudeSquared);
    if (magnitude <= maxMagnitude)
    {
        return force;
    }

    float scale = maxMagnitude / magnitude;
    return force * scale;
}
} // namespace

Vector2 getPotentialForce(float x, float y)
{
    const float upperLaneCenterY = 128.0f;
    const float rightLaneCenterX = 530.0f;
    const float lowerLaneCenterX = 330.0f;
    const Vector2 targetCenter(330.0f, 495.0f);

    Vector2 force(0.0f, 0.0f);

    if (x < 180.0f)
    {
        force.x += FIELD_DRIVE_STRENGTH;
    }
    else if (y < 170.0f)
    {
        force.x += FIELD_DRIVE_STRENGTH;
        force.y += (upperLaneCenterY - y) * FIELD_CENTERING_STRENGTH;
    }
    else if (x > 430.0f && y < 362.0f)
    {
        force.y += FIELD_DRIVE_STRENGTH;
        force.x += (rightLaneCenterX - x) * FIELD_CENTERING_STRENGTH;
    }
    else if (y < 450.0f)
    {
        force.y += FIELD_DRIVE_STRENGTH;
        force.x += (lowerLaneCenterX - x) * FIELD_CENTERING_STRENGTH;
    }
    else
    {
        force.x += (targetCenter.x - x) * FIELD_TARGET_STRENGTH;
        force.y += (targetCenter.y - y) * FIELD_TARGET_STRENGTH;
    }

    // Слабое глобальное притяжение к центру цели помогает полю быть
    // более плавным на переходах между сегментами коридора.
    force += (targetCenter - Vector2(x, y)) * FIELD_TARGET_STRENGTH;

    force.x = clampMagnitude(force.x, FIELD_MAX_FORCE);
    force.y = clampMagnitude(force.y, FIELD_MAX_FORCE);
    return limitForce(force, FIELD_MAX_FORCE);
}

Vector2 Simulation::computePairForce(const Particle &a, const Particle &b) const
{
    Vector2 displacement = a.position - b.position;
    float distanceSquared = displacement.x * displacement.x + displacement.y * displacement.y;

    if (distanceSquared == 0.0f)
    {
        return Vector2(0, 0);
    }

    float distance = std::sqrt(distanceSquared);
    if (distance > PAIR_FORCE_CUTOFF)
    {
        return Vector2(0, 0);
    }

    if (distance < MIN_PAIR_DISTANCE)
    {
        distance = MIN_PAIR_DISTANCE;
    }

    Vector2 direction = displacement * (1.0f / distance);
    float delta = distance - MORSE_EQUILIBRIUM_DISTANCE;
    float expTerm = std::exp(-MORSE_WIDTH * delta);

    // Потенциал Морзе:
    // U(r) = D * (exp(-2a(r-r0)) - 2 exp(-a(r-r0)))
    // Тогда сила:
    // F(r) = 2 a D * (exp(-2a(r-r0)) - exp(-a(r-r0))) * r_hat
    float scalarForce = 2.0f * MORSE_WIDTH * MORSE_DEPTH * (expTerm * expTerm - expTerm);

    return limitForce(direction * scalarForce, FIELD_MAX_FORCE);
}

void Simulation::applyExternalForces()
{
    for (auto &p : particles)
    {
        p.applyForce(getPotentialForce(p.position.x, p.position.y));
    }
}

void Simulation::applyInteractionForces()
{
    for (std::size_t i = 0; i < particles.size(); ++i)
    {
        for (std::size_t j = i + 1; j < particles.size(); ++j)
        {
            Vector2 force = computePairForce(particles[i], particles[j]);
            particles[i].applyForce(force);
            particles[j].applyForce(force * -1.0f);
        }
    }
}

void Simulation::accumulateForces()
{
    for (auto &p : particles)
    {
        p.clearForces();
    }

    applyExternalForces();
    applyInteractionForces();
}

Simulation::Simulation()
    : corridor(std::vector<Vector2>{
                   Vector2(90.0f, 40.0f),
                   Vector2(180.0f, 40.0f),
                   Vector2(180.0f, 98.0f),
                   Vector2(560.0f, 98.0f),
                   Vector2(560.0f, 362.0f),
                   Vector2(360.0f, 362.0f),
                   Vector2(360.0f, 450.0f),
                   Vector2(415.0f, 450.0f),
                   Vector2(415.0f, 540.0f),
                   Vector2(245.0f, 540.0f),
                   Vector2(245.0f, 450.0f),
                   Vector2(300.0f, 450.0f),
                   Vector2(300.0f, 302.0f),
                   Vector2(500.0f, 302.0f),
                   Vector2(500.0f, 158.0f),
                   Vector2(180.0f, 158.0f),
                   Vector2(180.0f, 220.0f),
                   Vector2(90.0f, 220.0f)},
               WALL_COLOR, WALL_THICKNESS),
      spawnZone(sf::FloatRect(100.0f, 50.0f, 70.0f, 160.0f), SPAWN_COLOR),
      targetZone(sf::FloatRect(260.0f, 465.0f, 140.0f, 60.0f), TARGET_COLOR),
      window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Modeling: Ideal Gas"),
      cameraController(static_cast<float>(WINDOW_WIDTH), static_cast<float>(WINDOW_HEIGHT),
                       CAMERA_ZOOM_STEP, CAMERA_MIN_ZOOM, CAMERA_MAX_ZOOM),
      width(WINDOW_WIDTH), height(WINDOW_HEIGHT),
      rng(std::random_device{}()),
      distPos(0, 1),
      distVel(-V_MAX, V_MAX)
{
    window.setFramerateLimit(60);
    initParticles();
}

void Simulation::initParticles()
{
    particles.clear();
    for (int i = 0; i < PARTICLE_COUNT; ++i)
    {
        Vector2 spawnPoint = spawnZone.randomPoint(rng, distPos, PARTICLE_RADIUS);
        float vx = distVel(rng);
        float vy = distVel(rng);
        particles.emplace_back(spawnPoint.x, spawnPoint.y, vx, vy);
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
            if (event.type == sf::Event::MouseButtonPressed &&
                event.mouseButton.button == sf::Mouse::Left)
            {
                cameraController.startDrag(sf::Mouse::getPosition(window));
            }
            if (event.type == sf::Event::MouseButtonReleased &&
                event.mouseButton.button == sf::Mouse::Left)
            {
                cameraController.stopDrag();
            }
            if (event.type == sf::Event::MouseMoved)
            {
                cameraController.dragTo(window, sf::Mouse::getPosition(window));
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space)
            {
                initParticles();
            }
            if (event.type == sf::Event::MouseWheelScrolled &&
                event.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel)
            {
                if (event.mouseWheelScroll.delta > 0.0f)
                {
                    cameraController.zoomAt(1.0f / CAMERA_ZOOM_STEP, window, sf::Mouse::getPosition(window));
                }
                else if (event.mouseWheelScroll.delta < 0.0f)
                {
                    cameraController.zoomAt(CAMERA_ZOOM_STEP, window, sf::Mouse::getPosition(window));
                }
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Add)
            {
                cameraController.zoomAt(1.0f / CAMERA_ZOOM_STEP, window, sf::Mouse::getPosition(window));
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Equal)
            {
                cameraController.zoomAt(1.0f / CAMERA_ZOOM_STEP, window, sf::Mouse::getPosition(window));
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Hyphen)
            {
                cameraController.zoomAt(CAMERA_ZOOM_STEP, window, sf::Mouse::getPosition(window));
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Subtract)
            {
                cameraController.zoomAt(CAMERA_ZOOM_STEP, window, sf::Mouse::getPosition(window));
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Num0)
            {
                cameraController.reset();
                cameraController.updateViewport(window);
            }
        }

        accumulateForces();

        for (auto &p : particles)
        {
            Vector2 previousPosition = p.position;
            p.update(DT);
            corridor.resolveCollision(p, previousPosition, PARTICLE_RADIUS);
            targetZone.processParticle(p, PARTICLE_RADIUS);
        }

        window.clear(BACKGROUND_COLOR);
        cameraController.updateViewport(window);
        window.setView(cameraController.getView());

        spawnZone.draw(window);
        targetZone.draw(window);
        corridor.draw(window);

        for (const auto &p : particles)
            window.draw(p.shape);

        window.display();
    }
}
