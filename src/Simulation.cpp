#include "Simulation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace
{
const char *WINDOW_TITLE = "Modeling: Ideal Gas";
constexpr int MIN_GUI_STEPS_PER_FRAME = 1;
constexpr int MAX_GUI_STEPS_PER_FRAME = 10000;
constexpr float PI = 3.14159265f;
constexpr int MAJOR_GRID_INTERVAL = 5;
constexpr std::size_t SOURCE_CIRCLE_SEGMENTS = 96;
constexpr std::size_t SOURCE_CAPSULE_ARC_SEGMENTS = 24;
constexpr std::size_t SOURCE_GRADIENT_RINGS = 18;
constexpr float HUD_MARGIN = 16.0f;
constexpr float HUD_PADDING_X = 14.0f;
constexpr float HUD_PADDING_Y = 10.0f;
constexpr unsigned int HUD_TEXT_SIZE = 24;
constexpr float HUD_PANEL_MIN_HEIGHT = 40.0f;

bool loadHudFont(sf::Font &font)
{
    constexpr std::array<const char *, 2> fontCandidates = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"};

    for (const char *fontPath : fontCandidates)
    {
        if (font.openFromFile(fontPath))
        {
            return true;
        }
    }

    return false;
}

sf::Color withAlpha(const sf::Color &color, std::uint8_t alpha)
{
    return sf::Color(color.r, color.g, color.b, alpha);
}

sf::Color getSourceColor(float strength)
{
    return strength < 0.0f ? VisualConfig::Visuals::ATTRACTIVE_SOURCE_COLOR
                           : VisualConfig::Visuals::REPULSIVE_SOURCE_COLOR;
}

sf::Color lerpColor(const sf::Color &from, const sf::Color &to, float t)
{
    const auto mixChannel = [t](std::uint8_t a, std::uint8_t b) -> std::uint8_t
    {
        return static_cast<std::uint8_t>(static_cast<float>(a) + (static_cast<float>(b) - static_cast<float>(a)) * t);
    };

    return sf::Color(
        mixChannel(from.r, to.r),
        mixChannel(from.g, to.g),
        mixChannel(from.b, to.b),
        mixChannel(from.a, to.a));
}

Vector2 normalizeOrZero(const Vector2 &value)
{
    const float lengthSquared = value.x * value.x + value.y * value.y;
    if (lengthSquared == 0.0f)
    {
        return Vector2(0.0f, 0.0f);
    }

    const float inverseLength = 1.0f / std::sqrt(lengthSquared);
    return value * inverseLength;
}

bool hasSegmentShape(const Config::Field::Source &source)
{
    return source.segmentStart.has_value() && source.segmentEnd.has_value();
}

std::vector<Vector2> buildCircleOutline(const Vector2 &center, float radius)
{
    std::vector<Vector2> outline;
    outline.reserve(SOURCE_CIRCLE_SEGMENTS + 1);

    for (std::size_t pointIndex = 0; pointIndex <= SOURCE_CIRCLE_SEGMENTS; ++pointIndex)
    {
        const float angle = static_cast<float>(pointIndex) / static_cast<float>(SOURCE_CIRCLE_SEGMENTS) * 2.0f * PI;
        outline.emplace_back(center.x + std::cos(angle) * radius, center.y + std::sin(angle) * radius);
    }

    return outline;
}

std::vector<Vector2> buildCapsuleOutline(const Vector2 &start, const Vector2 &end, float radius)
{
    const Vector2 direction = normalizeOrZero(end - start);
    if (direction.x == 0.0f && direction.y == 0.0f)
    {
        return buildCircleOutline(start, radius);
    }

    const float directionAngle = std::atan2(direction.y, direction.x);
    std::vector<Vector2> outline;
    outline.reserve((SOURCE_CAPSULE_ARC_SEGMENTS + 1) * 2 + 1);

    for (std::size_t step = 0; step <= SOURCE_CAPSULE_ARC_SEGMENTS; ++step)
    {
        const float t = static_cast<float>(step) / static_cast<float>(SOURCE_CAPSULE_ARC_SEGMENTS);
        const float angle = directionAngle + PI * 0.5f - t * PI;
        outline.emplace_back(end.x + std::cos(angle) * radius, end.y + std::sin(angle) * radius);
    }

    for (std::size_t step = 0; step <= SOURCE_CAPSULE_ARC_SEGMENTS; ++step)
    {
        const float t = static_cast<float>(step) / static_cast<float>(SOURCE_CAPSULE_ARC_SEGMENTS);
        const float angle = directionAngle - PI * 0.5f - t * PI;
        outline.emplace_back(start.x + std::cos(angle) * radius, start.y + std::sin(angle) * radius);
    }

    if (!outline.empty())
    {
        outline.push_back(outline.front());
    }

    return outline;
}

std::vector<Vector2> buildGlowOutline(const Config::Field::Source &source, float radius)
{
    if (!hasSegmentShape(source))
    {
        return buildCircleOutline(source.center, radius);
    }

    return buildCapsuleOutline(*source.segmentStart, *source.segmentEnd, radius);
}

float smoothGlowFalloff(float t)
{
    const float clampedT = std::clamp(t, 0.0f, 1.0f);
    const float inverse = 1.0f - clampedT;
    return std::pow(inverse, 1.65f);
}

sf::Color sampleGlowColor(const sf::Color &sourceColor, float intensity, float t)
{
    const float falloff = smoothGlowFalloff(t);
    const float centerBlend = std::pow(1.0f - t, 1.2f);
    const sf::Color blendedColor = lerpColor(
        sourceColor,
        VisualConfig::Visuals::SOURCE_CENTER_COLOR,
        std::clamp(0.35f + centerBlend * 0.65f, 0.0f, 1.0f));

    const float boostedIntensity = 0.45f + intensity * 0.85f;
    const float alpha = (78.0f + boostedIntensity * 120.0f) * falloff;
    return withAlpha(blendedColor, static_cast<std::uint8_t>(std::clamp(alpha, 0.0f, 235.0f)));
}

void drawSmoothSourceGlow(sf::RenderTarget &target,
                          const Config::Field::Source &source,
                          float radius,
                          const sf::Color &sourceColor,
                          float intensity)
{
    for (std::size_t ringIndex = 0; ringIndex < SOURCE_GRADIENT_RINGS; ++ringIndex)
    {
        const float innerT = static_cast<float>(ringIndex) / static_cast<float>(SOURCE_GRADIENT_RINGS);
        const float outerT = static_cast<float>(ringIndex + 1) / static_cast<float>(SOURCE_GRADIENT_RINGS);
        const float innerRadius = radius * innerT;
        const float outerRadius = radius * outerT;
        const sf::Color innerColor = sampleGlowColor(sourceColor, intensity, innerT);
        const sf::Color outerColor = sampleGlowColor(sourceColor, intensity, outerT);

        const std::vector<Vector2> innerOutline = buildGlowOutline(source, innerRadius);
        const std::vector<Vector2> outerOutline = buildGlowOutline(source, outerRadius);
        const std::size_t pointCount = std::min(innerOutline.size(), outerOutline.size());

        sf::VertexArray ring(sf::PrimitiveType::TriangleStrip, pointCount * 2);
        for (std::size_t pointIndex = 0; pointIndex < pointCount; ++pointIndex)
        {
            const std::size_t vertexIndex = pointIndex * 2;
            const Vector2 &innerPoint = innerOutline[pointIndex];
            const Vector2 &outerPoint = outerOutline[pointIndex];

            ring[vertexIndex].position = {innerPoint.x, innerPoint.y};
            ring[vertexIndex].color = innerColor;
            ring[vertexIndex + 1].position = {outerPoint.x, outerPoint.y};
            ring[vertexIndex + 1].color = outerColor;
        }

        target.draw(ring);
    }
}

sf::FloatRect getViewBounds(const sf::View &view)
{
    const sf::Vector2f viewSize = view.getSize();
    const sf::Vector2f viewCenter = view.getCenter();

    return sf::FloatRect(
        {viewCenter.x - viewSize.x * 0.5f, viewCenter.y - viewSize.y * 0.5f},
        viewSize);
}

void appendLine(sf::VertexArray &lines,
                const sf::Vector2f &start,
                const sf::Vector2f &end,
                const sf::Color &color)
{
    lines.append(sf::Vertex{start, color});
    lines.append(sf::Vertex{end, color});
}

void drawCappedSegment(sf::RenderTarget &target,
                       const Vector2 &start,
                       const Vector2 &end,
                       float thickness,
                       const sf::Color &color)
{
    const Vector2 delta = end - start;
    const float length = std::sqrt(delta.x * delta.x + delta.y * delta.y);
    if (length == 0.0f)
    {
        return;
    }

    const Vector2 direction = delta * (1.0f / length);
    const float halfThickness = thickness * 0.5f;
    const Vector2 extendedStart = start - direction * halfThickness;
    const float cappedLength = length + thickness;
    const float angle = std::atan2(direction.y, direction.x) * 180.0f / PI;

    sf::RectangleShape segment({cappedLength, thickness});
    segment.setOrigin({0.0f, halfThickness});
    segment.setPosition({extendedStart.x, extendedStart.y});
    segment.setRotation(sf::degrees(angle));
    segment.setFillColor(color);
    target.draw(segment);
}
} // namespace

Simulation::Simulation(const RunConfig &runConfig)
    : core(runConfig),
      window(sf::VideoMode({VisualConfig::Window::WIDTH, VisualConfig::Window::HEIGHT}), WINDOW_TITLE),
      cameraController(static_cast<float>(VisualConfig::Window::WIDTH), static_cast<float>(VisualConfig::Window::HEIGHT), VisualConfig::Camera::ZOOM_STEP, VisualConfig::Camera::MIN_ZOOM, VisualConfig::Camera::MAX_ZOOM),
      guiStepsPerFrame(std::clamp(runConfig.physicsStepsPerFrame, MIN_GUI_STEPS_PER_FRAME, MAX_GUI_STEPS_PER_FRAME)),
      paused(false),
      hudFontLoaded(loadHudFont(hudFont))
{
    window.setFramerateLimit(60);

    particleShapeTemplate.setRadius(Config::Simulation::PARTICLE_RADIUS);
    particleShapeTemplate.setFillColor(VisualConfig::Visuals::PARTICLE_COLOR);
    particleShapeTemplate.setOrigin({Config::Simulation::PARTICLE_RADIUS, Config::Simulation::PARTICLE_RADIUS});

    zoneShapeTemplate.setFillColor(sf::Color::Transparent);
    updateWindowTitle();
}

void Simulation::processEvents()
{
    while (const std::optional event = window.pollEvent())
    {
        handleEvent(*event);
    }
}

void Simulation::handleEvent(const sf::Event &event)
{
    if (event.is<sf::Event::Closed>())
    {
        window.close();
    }
    else if (const auto *mouseButtonPressed = event.getIf<sf::Event::MouseButtonPressed>())
    {
        if (mouseButtonPressed->button == sf::Mouse::Button::Left)
        {
            cameraController.startDrag(mouseButtonPressed->position);
        }
    }
    else if (const auto *mouseButtonReleased = event.getIf<sf::Event::MouseButtonReleased>())
    {
        if (mouseButtonReleased->button == sf::Mouse::Button::Left)
        {
            cameraController.stopDrag();
        }
    }
    else if (const auto *mouseMoved = event.getIf<sf::Event::MouseMoved>())
    {
        cameraController.dragTo(window, mouseMoved->position);
    }
    else if (const auto *keyPressed = event.getIf<sf::Event::KeyPressed>())
    {
        handleKeyPressed(*keyPressed);
    }
    else if (const auto *mouseWheelScrolled = event.getIf<sf::Event::MouseWheelScrolled>())
    {
        handleMouseWheelScrolled(*mouseWheelScrolled);
    }
}

void Simulation::handleKeyPressed(const sf::Event::KeyPressed &keyPressed)
{
    if (keyPressed.code == sf::Keyboard::Key::Space)
    {
        core.reset();
        paused = false;
        updateWindowTitle();
    }
    else if (keyPressed.code == sf::Keyboard::Key::Left)
    {
        slowDown();
    }
    else if (keyPressed.code == sf::Keyboard::Key::Right)
    {
        speedUp();
    }
    else if (keyPressed.code == sf::Keyboard::Key::P)
    {
        togglePause();
    }
    else if (keyPressed.code == sf::Keyboard::Key::Add ||
             keyPressed.code == sf::Keyboard::Key::Equal)
    {
        cameraController.zoomAt(1.0f / VisualConfig::Camera::ZOOM_STEP, window, sf::Mouse::getPosition(window));
    }
    else if (keyPressed.code == sf::Keyboard::Key::Hyphen ||
             keyPressed.code == sf::Keyboard::Key::Subtract)
    {
        cameraController.zoomAt(VisualConfig::Camera::ZOOM_STEP, window, sf::Mouse::getPosition(window));
    }
    else if (keyPressed.code == sf::Keyboard::Key::Num0)
    {
        cameraController.reset();
        cameraController.updateViewport(window);
    }
}

void Simulation::slowDown()
{
    guiStepsPerFrame = std::max(MIN_GUI_STEPS_PER_FRAME, guiStepsPerFrame / 2);
    updateWindowTitle();
}

void Simulation::speedUp()
{
    guiStepsPerFrame = std::min(MAX_GUI_STEPS_PER_FRAME, guiStepsPerFrame * 2);
    updateWindowTitle();
}

void Simulation::togglePause()
{
    paused = !paused;
    updateWindowTitle();
}

void Simulation::updateWindowTitle()
{
    std::string title = WINDOW_TITLE;
    title += " | GUI speed: ";
    title += std::to_string(guiStepsPerFrame);
    title += " dt-steps/frame";

    if (paused)
    {
        title += " | paused";
    }

    title += " | Left/Right speed, P pause";
    window.setTitle(title);
}

void Simulation::handleMouseWheelScrolled(const sf::Event::MouseWheelScrolled &mouseWheelScrolled)
{
    if (mouseWheelScrolled.wheel != sf::Mouse::Wheel::Vertical)
    {
        return;
    }

    if (mouseWheelScrolled.delta > 0.0f)
    {
        cameraController.zoomAt(1.0f / VisualConfig::Camera::ZOOM_STEP, window, mouseWheelScrolled.position);
    }
    else if (mouseWheelScrolled.delta < 0.0f)
    {
        cameraController.zoomAt(VisualConfig::Camera::ZOOM_STEP, window, mouseWheelScrolled.position);
    }
}

void Simulation::drawZone(const Zone &zone, const sf::Color &color)
{
    const Rect &bounds = zone.getBounds();
    zoneShapeTemplate.setSize({bounds.width, bounds.height});
    zoneShapeTemplate.setPosition({bounds.x, bounds.y});
    zoneShapeTemplate.setFillColor(color);
    window.draw(zoneShapeTemplate);
}

void Simulation::drawWorldGuides()
{
    const float gridSpacing = Config::World::GRID_UNIT;
    if (gridSpacing <= 0.0f)
    {
        return;
    }

    const sf::FloatRect viewBounds = getViewBounds(cameraController.getView());
    const float left = viewBounds.position.x;
    const float top = viewBounds.position.y;
    const float right = left + viewBounds.size.x;
    const float bottom = top + viewBounds.size.y;

    sf::VertexArray gridLines(sf::PrimitiveType::Lines);

    const float startX = std::floor(left / gridSpacing) * gridSpacing;
    const float endX = std::ceil(right / gridSpacing) * gridSpacing;
    for (float x = startX; x <= endX + gridSpacing * 0.5f; x += gridSpacing)
    {
        if (std::abs(x) < 0.001f)
        {
            continue;
        }

        const int gridIndex = static_cast<int>(std::lround(x / gridSpacing));
        const sf::Color color = (gridIndex % MAJOR_GRID_INTERVAL == 0)
                                    ? VisualConfig::Visuals::GRID_MAJOR_COLOR
                                    : VisualConfig::Visuals::GRID_MINOR_COLOR;
        appendLine(gridLines, {x, top}, {x, bottom}, color);
    }

    const float startY = std::floor(top / gridSpacing) * gridSpacing;
    const float endY = std::ceil(bottom / gridSpacing) * gridSpacing;
    for (float y = startY; y <= endY + gridSpacing * 0.5f; y += gridSpacing)
    {
        if (std::abs(y) < 0.001f)
        {
            continue;
        }

        const int gridIndex = static_cast<int>(std::lround(y / gridSpacing));
        const sf::Color color = (gridIndex % MAJOR_GRID_INTERVAL == 0)
                                    ? VisualConfig::Visuals::GRID_MAJOR_COLOR
                                    : VisualConfig::Visuals::GRID_MINOR_COLOR;
        appendLine(gridLines, {left, y}, {right, y}, color);
    }

    window.draw(gridLines);

    sf::VertexArray axes(sf::PrimitiveType::Lines);
    appendLine(axes, {left, 0.0f}, {right, 0.0f}, VisualConfig::Visuals::X_AXIS_COLOR);
    appendLine(axes, {0.0f, top}, {0.0f, bottom}, VisualConfig::Visuals::Y_AXIS_COLOR);
    appendLine(
        axes,
        {-VisualConfig::Visuals::ORIGIN_MARKER_HALF_SIZE, 0.0f},
        {VisualConfig::Visuals::ORIGIN_MARKER_HALF_SIZE, 0.0f},
        VisualConfig::Visuals::ORIGIN_COLOR);
    appendLine(
        axes,
        {0.0f, -VisualConfig::Visuals::ORIGIN_MARKER_HALF_SIZE},
        {0.0f, VisualConfig::Visuals::ORIGIN_MARKER_HALF_SIZE},
        VisualConfig::Visuals::ORIGIN_COLOR);
    window.draw(axes);

    sf::CircleShape originMarker(VisualConfig::Visuals::ORIGIN_MARKER_RADIUS);
    originMarker.setOrigin({VisualConfig::Visuals::ORIGIN_MARKER_RADIUS, VisualConfig::Visuals::ORIGIN_MARKER_RADIUS});
    originMarker.setPosition({0.0f, 0.0f});
    originMarker.setFillColor(VisualConfig::Visuals::ORIGIN_COLOR);
    window.draw(originMarker);
}

void Simulation::drawBoundary(const Corridor &boundary, const sf::Color &color)
{
    const std::vector<Vector2> &outline = boundary.getOutline();

    for (std::size_t i = 0; i < outline.size(); ++i)
    {
        const Vector2 &start = outline[i];
        const Vector2 &end = outline[(i + 1) % outline.size()];
        drawCappedSegment(window, start, end, VisualConfig::Visuals::WALL_THICKNESS, color);
    }
}

void Simulation::drawFieldSources()
{
    const auto &sources = core.getFieldModel().sources.sources;
    if (sources.empty())
    {
        return;
    }

    float maxAbsStrength = 0.0f;
    for (const auto &source : sources)
    {
        maxAbsStrength = std::max(maxAbsStrength, std::abs(source.strength));
    }

    maxAbsStrength = std::max(maxAbsStrength, 1.0f);

    for (const auto &source : sources)
    {
        drawFieldSource(source, maxAbsStrength);
    }
}

void Simulation::drawFieldSource(const Config::Field::Source &source, float maxAbsStrength)
{
    const sf::Color sourceColor = getSourceColor(source.strength);
    const float intensity = std::clamp(std::abs(source.strength) / maxAbsStrength, 0.25f, 1.0f);
    const float baseRadius = source.sigma * VisualConfig::Visuals::SOURCE_INFLUENCE_RADIUS_MULTIPLIER;
    drawSmoothSourceGlow(window, source, baseRadius, sourceColor, intensity);
}

void Simulation::drawTargetCounter()
{
    if (!hudFontLoaded)
    {
        return;
    }

    const SimulationStats &stats = core.getStats();
    const std::string counterText = std::to_string(stats.uniqueTargetHits) + "/" +
                                    std::to_string(stats.reachedTarget.size());

    sf::Text text(hudFont, counterText, HUD_TEXT_SIZE);
    text.setFillColor(sf::Color(245, 248, 255));

    const sf::FloatRect textBounds = text.getLocalBounds();
    const sf::Vector2f panelSize(
        textBounds.size.x + HUD_PADDING_X * 2.0f,
        std::max(HUD_PANEL_MIN_HEIGHT, textBounds.size.y + HUD_PADDING_Y * 2.0f));

    sf::RectangleShape panel(panelSize);
    panel.setPosition(
        {static_cast<float>(window.getSize().x) - HUD_MARGIN - panelSize.x, HUD_MARGIN});
    panel.setFillColor(sf::Color(16, 20, 26, 215));
    panel.setOutlineThickness(1.5f);
    panel.setOutlineColor(withAlpha(VisualConfig::Visuals::TARGET_COLOR, 200));

    const sf::View worldView = window.getView();
    const sf::View hudView(sf::FloatRect(
        {0.0f, 0.0f},
        {static_cast<float>(window.getSize().x), static_cast<float>(window.getSize().y)}));
    window.setView(hudView);
    window.draw(panel);

    text.setPosition(
        {panel.getPosition().x + HUD_PADDING_X - textBounds.position.x,
         panel.getPosition().y + (panelSize.y - textBounds.size.y) * 0.5f - textBounds.position.y});
    window.draw(text);
    window.setView(worldView);
}

void Simulation::update()
{
    if (paused)
    {
        return;
    }

    core.step(guiStepsPerFrame);
}

void Simulation::render()
{
    window.clear(VisualConfig::Window::BACKGROUND_COLOR);
    cameraController.updateViewport(window);
    window.setView(cameraController.getView());

    drawWorldGuides();
    drawFieldSources();
    drawZone(core.getSpawnZone(), VisualConfig::Visuals::SPAWN_COLOR);
    drawZone(core.getTargetZone(), VisualConfig::Visuals::TARGET_COLOR);
    drawBoundary(core.getCorridor(), VisualConfig::Visuals::WALL_COLOR);
    for (const auto &obstacle : core.getObstacles())
    {
        drawBoundary(obstacle, VisualConfig::Visuals::WALL_COLOR);
    }

    for (const auto &particle : core.getParticles())
    {
        particleShapeTemplate.setPosition({particle.position.x, particle.position.y});
        window.draw(particleShapeTemplate);
    }

    drawTargetCounter();
    window.display();
}

void Simulation::run()
{
    while (window.isOpen())
    {
        processEvents();
        update();
        render();
    }
}
