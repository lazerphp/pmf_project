#include "Simulation.h"

namespace Config
{
// Window and rendering
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const sf::Color BACKGROUND_COLOR(12, 14, 18);

// Particles and integration
const int PARTICLE_COUNT = 50;
const int PHYSICS_STEPS_PER_FRAME = 5;
const float DT = 0.001f;
const float PARTICLE_RADIUS = 5.0f;
const float V_MAX = 200.0f;

// Lennard-Jones interaction
const float LJ_EPSILON = 900.0f;
const float LJ_SIGMA = (2.0f * PARTICLE_RADIUS) / 1.122462048f;
const float LJ_CUTOFF = 2.5f * LJ_SIGMA;
const float MIN_PAIR_DISTANCE = 0.6f * LJ_SIGMA;

// Spawn constraints
const float SPAWN_MIN_DISTANCE = 2.0f * PARTICLE_RADIUS;
const int SPAWN_MAX_ATTEMPTS = 200;

// External field
const float FIELD_START_DRIVE_X = 18.0f;
const float FIELD_TOP_DRIVE_X = 18.0f;
const float FIELD_TOP_CENTERING_Y = 0.12f;
const float FIELD_RIGHT_DRIVE_Y = 18.0f;
const float FIELD_RIGHT_CENTERING_X = 0.12f;
const float FIELD_BOTTOM_DRIVE_Y = 18.0f;
const float FIELD_BOTTOM_CENTERING_X = 0.12f;
const float FIELD_TARGET_ATTRACTION_X = 0.035f;
const float FIELD_TARGET_ATTRACTION_Y = 0.035f;
const float FIELD_GLOBAL_TARGET_ATTRACTION = 0.035f;
const float FIELD_MAX_FORCE = 160.0f;

// Camera
const float CAMERA_ZOOM_STEP = 1.1f;
const float CAMERA_MIN_ZOOM = 0.35f;
const float CAMERA_MAX_ZOOM = 4.0f;

// Visuals
const sf::Color WALL_COLOR(255, 255, 255);
const float WALL_THICKNESS = 4.0f;
const sf::Color SPAWN_COLOR(150, 210, 40, 150);
const sf::Color TARGET_COLOR(220, 50, 50, 150);
} // namespace Config

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
    const float upperLaneCenterY = 130.0f;
    const float rightLaneCenterX = 530.0f;
    const float lowerLaneCenterX = 330.0f;
    const Vector2 targetCenter(330.0f, 495.0f);

    Vector2 force(0.0f, 0.0f);

    if (x < 180.0f)
    {
        force.x += Config::FIELD_START_DRIVE_X;
    }
    else if (y < 170.0f)
    {
        force.x += Config::FIELD_TOP_DRIVE_X;
        force.y += (upperLaneCenterY - y) * Config::FIELD_TOP_CENTERING_Y;
    }
    else if (x > 430.0f && y < 362.0f)
    {
        force.y += Config::FIELD_RIGHT_DRIVE_Y;
        force.x += (rightLaneCenterX - x) * Config::FIELD_RIGHT_CENTERING_X;
    }
    else if (y < 450.0f)
    {
        force.y += Config::FIELD_BOTTOM_DRIVE_Y;
        force.x += (lowerLaneCenterX - x) * Config::FIELD_BOTTOM_CENTERING_X;
    }
    else
    {
        force.x += (targetCenter.x - x) * Config::FIELD_TARGET_ATTRACTION_X;
        force.y += (targetCenter.y - y) * Config::FIELD_TARGET_ATTRACTION_Y;
    }

    // Слабое глобальное притяжение к центру цели помогает полю быть
    // более плавным на переходах между сегментами коридора.
    force += (targetCenter - Vector2(x, y)) * Config::FIELD_GLOBAL_TARGET_ATTRACTION;

    force.x = clampMagnitude(force.x, Config::FIELD_MAX_FORCE);
    force.y = clampMagnitude(force.y, Config::FIELD_MAX_FORCE);
    return limitForce(force, Config::FIELD_MAX_FORCE);
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
    if (distance > Config::LJ_CUTOFF)
    {
        return Vector2(0, 0);
    }

    if (distance < Config::MIN_PAIR_DISTANCE)
    {
        distance = Config::MIN_PAIR_DISTANCE;
    }

    Vector2 direction = displacement * (1.0f / distance);
    float sigmaOverR = Config::LJ_SIGMA / distance;
    float sigmaOverR2 = sigmaOverR * sigmaOverR;
    float sigmaOverR6 = sigmaOverR2 * sigmaOverR2 * sigmaOverR2;
    float sigmaOverR12 = sigmaOverR6 * sigmaOverR6;

    // Потенциал Леннарда-Джонса:
    // U(r) = 4 epsilon * ((sigma/r)^12 - (sigma/r)^6)
    // Сила:
    // F(r) = 24 epsilon / r * (2 (sigma/r)^12 - (sigma/r)^6) * r_hat
    float scalarForce = (24.0f * Config::LJ_EPSILON / distance) * (2.0f * sigmaOverR12 - sigmaOverR6);

    return direction * scalarForce;
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
                   Vector2(180.0f, 100.0f),
                   Vector2(560.0f, 100.0f),
                   Vector2(560.0f, 362.0f),
                   Vector2(360.0f, 362.0f),
                   Vector2(360.0f, 450.0f),
                   Vector2(420.0f, 450.0f),
                   Vector2(420.0f, 540.0f),
                   Vector2(240.0f, 540.0f),
                   Vector2(240.0f, 450.0f),
                   Vector2(300.0f, 450.0f),
                   Vector2(300.0f, 302.0f),
                   Vector2(500.0f, 302.0f),
                   Vector2(500.0f, 160.0f),
                   Vector2(180.0f, 160.0f),
                   Vector2(180.0f, 220.0f),
                   Vector2(90.0f, 220.0f)},
               Config::WALL_COLOR, Config::WALL_THICKNESS),
      spawnZone(sf::FloatRect(90.0f, 40.0f, 90.0f, 180.0f), Config::SPAWN_COLOR),
      targetZone(sf::FloatRect(240.0f, 450.0f, 180.0f, 90.0f), Config::TARGET_COLOR),
      window(sf::VideoMode(Config::WINDOW_WIDTH, Config::WINDOW_HEIGHT), "Modeling: Ideal Gas"),
      cameraController(static_cast<float>(Config::WINDOW_WIDTH), static_cast<float>(Config::WINDOW_HEIGHT),
                       Config::CAMERA_ZOOM_STEP, Config::CAMERA_MIN_ZOOM, Config::CAMERA_MAX_ZOOM),
      width(Config::WINDOW_WIDTH), height(Config::WINDOW_HEIGHT),
      rng(std::random_device{}()),
      distPos(0, 1),
      distVel(-Config::V_MAX, Config::V_MAX)
{
    window.setFramerateLimit(60);
    initParticles();
}

void Simulation::initParticles()
{
    particles.clear();
    for (int i = 0; i < Config::PARTICLE_COUNT; ++i)
    {
        Vector2 spawnPoint;
        bool foundValidPosition = false;

        for (int attempt = 0; attempt < Config::SPAWN_MAX_ATTEMPTS; ++attempt)
        {
            spawnPoint = spawnZone.randomPoint(rng, distPos, Config::PARTICLE_RADIUS);
            foundValidPosition = true;

            for (const auto &existingParticle : particles)
            {
                Vector2 delta = spawnPoint - existingParticle.position;
                float distanceSquared = delta.x * delta.x + delta.y * delta.y;

                if (distanceSquared < Config::SPAWN_MIN_DISTANCE * Config::SPAWN_MIN_DISTANCE)
                {
                    foundValidPosition = false;
                    break;
                }
            }

            if (foundValidPosition)
            {
                break;
            }
        }

        if (!foundValidPosition)
        {
            break;
        }

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
                    cameraController.zoomAt(1.0f / Config::CAMERA_ZOOM_STEP, window, sf::Mouse::getPosition(window));
                }
                else if (event.mouseWheelScroll.delta < 0.0f)
                {
                    cameraController.zoomAt(Config::CAMERA_ZOOM_STEP, window, sf::Mouse::getPosition(window));
                }
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Add)
            {
                cameraController.zoomAt(1.0f / Config::CAMERA_ZOOM_STEP, window, sf::Mouse::getPosition(window));
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Equal)
            {
                cameraController.zoomAt(1.0f / Config::CAMERA_ZOOM_STEP, window, sf::Mouse::getPosition(window));
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Hyphen)
            {
                cameraController.zoomAt(Config::CAMERA_ZOOM_STEP, window, sf::Mouse::getPosition(window));
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Subtract)
            {
                cameraController.zoomAt(Config::CAMERA_ZOOM_STEP, window, sf::Mouse::getPosition(window));
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Num0)
            {
                cameraController.reset();
                cameraController.updateViewport(window);
            }
        }

        for (int step = 0; step < Config::PHYSICS_STEPS_PER_FRAME; ++step)
        {
            accumulateForces();

            for (auto &p : particles)
            {
                Vector2 previousPosition = p.position;
                p.update(Config::DT);
                corridor.resolveCollision(p, previousPosition, Config::PARTICLE_RADIUS);
                targetZone.processParticle(p, Config::PARTICLE_RADIUS);
            }
        }

        window.clear(Config::BACKGROUND_COLOR);
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
