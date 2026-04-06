#include "Simulation.h"

#include <cmath>
#include <optional>

namespace
{
    struct PotentialSample
    {
        float value;
        Vector2 gradient;
    };

    struct RadialPotentialSample
    {
        float value;
        float radialDerivative;
    };

    // Ограничить величину
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

    // Ограничить вектор силы
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

    PotentialSample getExternalPotentialSample(float x, float y)
    {
        float value = 0.0f;
        Vector2 gradient(0.0f, 0.0f);

        if (x < Config::Field::TOP_SEGMENT_X_MAX)
        {
            value += -Config::Field::START_DRIVE_X * x;
            gradient.x += -Config::Field::START_DRIVE_X;
        }
        else if (y < Config::Field::TOP_SEGMENT_Y_MAX)
        {
            value += -Config::Field::TOP_DRIVE_X * x;
            value += 0.5f * Config::Field::TOP_CENTERING_Y * (y - Config::Field::UPPER_LANE_CENTER_Y) *
                     (y - Config::Field::UPPER_LANE_CENTER_Y);
            gradient.x += -Config::Field::TOP_DRIVE_X;
            gradient.y += Config::Field::TOP_CENTERING_Y * (y - Config::Field::UPPER_LANE_CENTER_Y);
        }
        else if (x > Config::Field::RIGHT_SEGMENT_X_MIN && y < Config::Field::RIGHT_SEGMENT_Y_MAX)
        {
            value += -Config::Field::RIGHT_DRIVE_Y * y;
            value += 0.5f * Config::Field::RIGHT_CENTERING_X * (x - Config::Field::RIGHT_LANE_CENTER_X) *
                     (x - Config::Field::RIGHT_LANE_CENTER_X);
            gradient.x += Config::Field::RIGHT_CENTERING_X * (x - Config::Field::RIGHT_LANE_CENTER_X);
            gradient.y += -Config::Field::RIGHT_DRIVE_Y;
        }
        else if (y < Config::Field::LOWER_SEGMENT_Y_MAX)
        {
            value += -Config::Field::BOTTOM_DRIVE_Y * y;
            value += 0.5f * Config::Field::BOTTOM_CENTERING_X * (x - Config::Field::LOWER_LANE_CENTER_X) *
                     (x - Config::Field::LOWER_LANE_CENTER_X);
            gradient.x += Config::Field::BOTTOM_CENTERING_X * (x - Config::Field::LOWER_LANE_CENTER_X);
            gradient.y += -Config::Field::BOTTOM_DRIVE_Y;
        }
        else
        {
            value += 0.5f * Config::Field::TARGET_ATTRACTION_X * (x - Config::Field::TARGET_CENTER.x) *
                     (x - Config::Field::TARGET_CENTER.x);
            value += 0.5f * Config::Field::TARGET_ATTRACTION_Y * (y - Config::Field::TARGET_CENTER.y) *
                     (y - Config::Field::TARGET_CENTER.y);
            gradient.x += Config::Field::TARGET_ATTRACTION_X * (x - Config::Field::TARGET_CENTER.x);
            gradient.y += Config::Field::TARGET_ATTRACTION_Y * (y - Config::Field::TARGET_CENTER.y);
        }

        // Слабое глобальное притяжение к центру цели  для плавности
        Vector2 targetDelta(x - Config::Field::TARGET_CENTER.x, y - Config::Field::TARGET_CENTER.y);
        value += 0.5f * Config::Field::GLOBAL_TARGET_ATTRACTION *
                 (targetDelta.x * targetDelta.x + targetDelta.y * targetDelta.y);
        gradient += targetDelta * Config::Field::GLOBAL_TARGET_ATTRACTION;

        return PotentialSample{value, gradient};
    }

    RadialPotentialSample getPairPotentialSample(float distance)
    {
        float sigmaOverR = Config::LennardJones::sigma() / distance;
        float sigmaOverR2 = sigmaOverR * sigmaOverR;
        float sigmaOverR6 = sigmaOverR2 * sigmaOverR2 * sigmaOverR2;
        float sigmaOverR12 = sigmaOverR6 * sigmaOverR6;

        // Единая точка задания парного потенциала и его радиальной производной.
        // Если формула U(r) изменится, достаточно обновить только это место.
        float value = 4.0f * Config::LennardJones::EPSILON * (sigmaOverR12 - sigmaOverR6);
        float radialDerivative =
            (24.0f * Config::LennardJones::EPSILON / distance) * (sigmaOverR6 - 2.0f * sigmaOverR12);

        return RadialPotentialSample{value, radialDerivative};
    }

    Vector2 getPairPotentialGradient(const Vector2 &displacement, float distance)
    {
        Vector2 direction = displacement * (1.0f / distance);
        RadialPotentialSample sample = getPairPotentialSample(distance);

        // grad U(r) = dU/dr * r_hat, где r_hat = displacement / |displacement|
        return direction * sample.radialDerivative;
    }
} // namespace

Vector2 getPotentialForce(float x, float y)
{
    PotentialSample sample = getExternalPotentialSample(x, y);
    Vector2 force = sample.gradient * -1.0f;

    force.x = clampMagnitude(force.x, Config::Field::MAX_FORCE);
    force.y = clampMagnitude(force.y, Config::Field::MAX_FORCE);
    return limitForce(force, Config::Field::MAX_FORCE);
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
    if (distance > Config::LennardJones::cutoff())
    {
        return Vector2(0, 0);
    }

    if (distance < Config::LennardJones::minPairDistance())
    {
        distance = Config::LennardJones::minPairDistance();
    }

    // Для центрального потенциала U(r):
    // grad U = (dU/dr) * r_hat, а сила F = -grad U.
    // Явно вычисляем U(r) отдельной функцией выше, а здесь используем его градиент.
    Vector2 gradient = getPairPotentialGradient(displacement, distance);
    return gradient * -1.0f;
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
               Config::Visuals::WALL_COLOR, Config::Visuals::WALL_THICKNESS),
      spawnZone(sf::FloatRect({90.0f, 40.0f}, {90.0f, 180.0f}), Config::Visuals::SPAWN_COLOR),
      targetZone(sf::FloatRect({240.0f, 450.0f}, {180.0f, 90.0f}), Config::Visuals::TARGET_COLOR),
      window(sf::VideoMode({Config::Window::WIDTH, Config::Window::HEIGHT}), "Modeling: Ideal Gas"),
      cameraController(static_cast<float>(Config::Window::WIDTH), static_cast<float>(Config::Window::HEIGHT), Config::Camera::ZOOM_STEP, Config::Camera::MIN_ZOOM, Config::Camera::MAX_ZOOM),
      width(Config::Window::WIDTH),
      height(Config::Window::HEIGHT),
      rng(std::random_device{}()),
      distPos(0, 1),
      distVel(-Config::Simulation::V_MAX, Config::Simulation::V_MAX)
{
    window.setFramerateLimit(60);
    initParticles();
}

void Simulation::initParticles()
{
    particles.clear();
    for (int i = 0; i < Config::Simulation::PARTICLE_COUNT; ++i)
    {
        Vector2 spawnPoint;
        bool foundValidPosition = false;

        for (int attempt = 0; attempt < Config::Spawn::MAX_ATTEMPTS; ++attempt)
        {
            spawnPoint = spawnZone.randomPoint(rng, distPos, Config::Simulation::PARTICLE_RADIUS);
            foundValidPosition = true;

            for (const auto &existingParticle : particles)
            {
                Vector2 delta = spawnPoint - existingParticle.position;
                float distanceSquared = delta.x * delta.x + delta.y * delta.y;

                if (distanceSquared < Config::Spawn::minDistance() * Config::Spawn::minDistance())
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
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
            else if (const auto *mouseButtonPressed = event->getIf<sf::Event::MouseButtonPressed>())
            {
                if (mouseButtonPressed->button == sf::Mouse::Button::Left)
                {
                    cameraController.startDrag(mouseButtonPressed->position);
                }
            }
            else if (const auto *mouseButtonReleased = event->getIf<sf::Event::MouseButtonReleased>())
            {
                if (mouseButtonReleased->button == sf::Mouse::Button::Left)
                {
                    cameraController.stopDrag();
                }
            }
            else if (const auto *mouseMoved = event->getIf<sf::Event::MouseMoved>())
            {
                cameraController.dragTo(window, mouseMoved->position);
            }
            else if (const auto *keyPressed = event->getIf<sf::Event::KeyPressed>())
            {
                if (keyPressed->code == sf::Keyboard::Key::Space)
                {
                    initParticles();
                }
                else if (keyPressed->code == sf::Keyboard::Key::Add ||
                         keyPressed->code == sf::Keyboard::Key::Equal)
                {
                    cameraController.zoomAt(1.0f / Config::Camera::ZOOM_STEP, window, sf::Mouse::getPosition(window));
                }
                else if (keyPressed->code == sf::Keyboard::Key::Hyphen ||
                         keyPressed->code == sf::Keyboard::Key::Subtract)
                {
                    cameraController.zoomAt(Config::Camera::ZOOM_STEP, window, sf::Mouse::getPosition(window));
                }
                else if (keyPressed->code == sf::Keyboard::Key::Num0)
                {
                    cameraController.reset();
                    cameraController.updateViewport(window);
                }
            }
            else if (const auto *mouseWheelScrolled = event->getIf<sf::Event::MouseWheelScrolled>())
            {
                if (mouseWheelScrolled->wheel == sf::Mouse::Wheel::Vertical)
                {
                    if (mouseWheelScrolled->delta > 0.0f)
                    {
                        cameraController.zoomAt(1.0f / Config::Camera::ZOOM_STEP, window, mouseWheelScrolled->position);
                    }
                    else if (mouseWheelScrolled->delta < 0.0f)
                    {
                        cameraController.zoomAt(Config::Camera::ZOOM_STEP, window, mouseWheelScrolled->position);
                    }
                }
            }
        }

        for (int step = 0; step < Config::Simulation::PHYSICS_STEPS_PER_FRAME; ++step)
        {
            accumulateForces();

            for (auto &p : particles)
            {
                Vector2 previousPosition = p.position;
                p.update(Config::Simulation::DT);
                corridor.resolveCollision(p, previousPosition, Config::Simulation::PARTICLE_RADIUS);
                targetZone.processParticle(p, Config::Simulation::PARTICLE_RADIUS);
            }
        }

        window.clear(Config::Window::BACKGROUND_COLOR);
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
