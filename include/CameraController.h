#ifndef CAMERACONTROLLER_H
#define CAMERACONTROLLER_H

#include <SFML/Graphics.hpp>

class CameraController
{
private:
    sf::View view;
    float logicalWidth;
    float logicalHeight;
    float zoomLevel;
    float zoomStep;
    float minZoomLevel;
    float maxZoomLevel;
    bool isDragging;
    sf::Vector2i lastDragPixel;

public:
    CameraController();
    CameraController(float logicalWidth, float logicalHeight, float zoomStep, float minZoomLevel, float maxZoomLevel);

    void updateViewport(const sf::RenderWindow &window);
    void zoomAt(float factor, const sf::RenderWindow &window, const sf::Vector2i &pixel);
    void startDrag(const sf::Vector2i &pixel);
    void stopDrag();
    void dragTo(const sf::RenderWindow &window, const sf::Vector2i &pixel);
    void reset();

    const sf::View &getView() const;
};

#endif // CAMERACONTROLLER_H
