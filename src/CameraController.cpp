#include "CameraController.h"

CameraController::CameraController()
    : CameraController(800.0f, 600.0f, 1.1f, 0.35f, 4.0f) {}

CameraController::CameraController(float logicalWidth_, float logicalHeight_, float zoomStep_, float minZoomLevel_, float maxZoomLevel_)
    : view(sf::FloatRect(0.0f, 0.0f, logicalWidth_, logicalHeight_)),
      logicalWidth(logicalWidth_),
      logicalHeight(logicalHeight_),
      zoomLevel(1.0f),
      zoomStep(zoomStep_),
      minZoomLevel(minZoomLevel_),
      maxZoomLevel(maxZoomLevel_),
      isDragging(false),
      lastDragPixel(0, 0)
{
    view.setCenter(logicalWidth * 0.5f, logicalHeight * 0.5f);
}

void CameraController::updateViewport(const sf::RenderWindow &window)
{
    unsigned int winWidth = window.getSize().x;
    unsigned int winHeight = window.getSize().y;
    float windowRatio = static_cast<float>(winWidth) / static_cast<float>(winHeight);
    float logicalRatio = logicalWidth / logicalHeight;

    sf::FloatRect viewport;

    if (windowRatio > logicalRatio)
    {
        float newWidth = logicalRatio * winHeight;
        viewport.left = (winWidth - newWidth) / 2.0f / winWidth;
        viewport.top = 0.0f;
        viewport.width = newWidth / winWidth;
        viewport.height = 1.0f;
    }
    else
    {
        float newHeight = winWidth / logicalRatio;
        viewport.left = 0.0f;
        viewport.top = (winHeight - newHeight) / 2.0f / winHeight;
        viewport.width = 1.0f;
        viewport.height = newHeight / winHeight;
    }

    view.setSize(logicalWidth * zoomLevel, logicalHeight * zoomLevel);
    view.setViewport(viewport);
}

void CameraController::zoomAt(float factor, const sf::RenderWindow &window, const sf::Vector2i &pixel)
{
    float newZoomLevel = zoomLevel * factor;

    if (newZoomLevel < minZoomLevel || newZoomLevel > maxZoomLevel)
    {
        return;
    }

    sf::Vector2f beforeZoom = window.mapPixelToCoords(pixel, view);
    zoomLevel = newZoomLevel;
    updateViewport(window);
    sf::Vector2f afterZoom = window.mapPixelToCoords(pixel, view);
    view.move(beforeZoom - afterZoom);
}

void CameraController::startDrag(const sf::Vector2i &pixel)
{
    isDragging = true;
    lastDragPixel = pixel;
}

void CameraController::stopDrag()
{
    isDragging = false;
}

void CameraController::dragTo(const sf::RenderWindow &window, const sf::Vector2i &pixel)
{
    if (!isDragging)
    {
        return;
    }

    sf::Vector2f previousWorld = window.mapPixelToCoords(lastDragPixel, view);
    sf::Vector2f currentWorld = window.mapPixelToCoords(pixel, view);
    view.move(previousWorld - currentWorld);
    lastDragPixel = pixel;
}

void CameraController::reset()
{
    zoomLevel = 1.0f;
    view.setCenter(logicalWidth * 0.5f, logicalHeight * 0.5f);
}

const sf::View &CameraController::getView() const
{
    return view;
}
