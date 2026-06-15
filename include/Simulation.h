#ifndef SIMULATION_H
#define SIMULATION_H

#include <SFML/Graphics.hpp>

#include "CameraController.h"
#include "RunConfig.h"
#include "SimulationCore.h"
#include "VisualConfig.h"

class Simulation
{
private:
    SimulationCore core;
    sf::RenderWindow window;
    CameraController cameraController;
    sf::CircleShape particleShapeTemplate;
    sf::RectangleShape zoneShapeTemplate;
    sf::Font hudFont;
    int guiStepsPerFrame;
    bool paused;
    bool hudFontLoaded;

    void processEvents();
    void handleEvent(const sf::Event &event);
    void handleKeyPressed(const sf::Event::KeyPressed &keyPressed);
    void handleMouseWheelScrolled(const sf::Event::MouseWheelScrolled &mouseWheelScrolled);
    void drawWorldGuides();
    void drawZone(const Zone &zone, const sf::Color &color);
    void drawBoundary(const Corridor &boundary, const sf::Color &color);
    void drawFieldSources();
    void drawFieldSource(const Config::Field::Source &source, float maxAbsStrength);
    void drawTargetCounter();
    void slowDown();
    void speedUp();
    void togglePause();
    void syncPauseStateWithRunLimits();
    void updateWindowTitle();
    void update();
    void render();

public:
    explicit Simulation(const RunConfig &runConfig);
    void run();
};

#endif // SIMULATION_H
