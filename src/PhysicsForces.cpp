#include "PhysicsForces.h"

#include <algorithm>
#include <cmath>

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

    struct ShapeDistanceSample
    {
        float distance;
        Vector2 direction;
    };

    ShapeDistanceSample getPointDistanceSample(const Vector2 &delta)
    {
        const float distanceSquared = delta.x * delta.x + delta.y * delta.y;
        if (distanceSquared == 0.0f)
        {
            return ShapeDistanceSample{0.0f, Vector2(0.0f, 0.0f)};
        }

        const float distance = std::sqrt(distanceSquared);
        return ShapeDistanceSample{distance, delta * (1.0f / distance)};
    }

    ShapeDistanceSample getSegmentDistanceSample(const Vector2 &position, const Config::Field::Source &source)
    {
        if (!source.segmentStart.has_value() || !source.segmentEnd.has_value())
        {
            return getPointDistanceSample(position - source.center);
        }

        const Vector2 &start = *source.segmentStart;
        const Vector2 &end = *source.segmentEnd;
        const Vector2 segment = end - start;
        const float lengthSquared = segment.x * segment.x + segment.y * segment.y;
        if (lengthSquared == 0.0f)
        {
            return getPointDistanceSample(position - source.center);
        }

        const Vector2 fromStart = position - start;
        const float projection = fromStart.x * segment.x + fromStart.y * segment.y;
        const float t = std::clamp(projection / lengthSquared, 0.0f, 1.0f);
        const Vector2 closestPoint = start + segment * t;
        return getPointDistanceSample(position - closestPoint);
    }

    PotentialSample getSourcesPotentialSample(float x, float y, const Config::Field::SourcesParams &sourcesParams)
    {
        float value = 0.0f;
        Vector2 gradient(0.0f, 0.0f);
        const Vector2 position(x, y);

        for (const auto &source : sourcesParams.sources)
        {
            if (source.sigma <= 0.0f)
            {
                continue;
            }

            const float sigmaSquared = source.sigma * source.sigma;
            const ShapeDistanceSample shapeDistance = getSegmentDistanceSample(position, source);
            const float distanceSquared = shapeDistance.distance * shapeDistance.distance;
            const float exponent = -distanceSquared / (2.0f * sigmaSquared);
            const float base = source.strength * std::exp(exponent);
            value += base;
            gradient += shapeDistance.direction * (-base * shapeDistance.distance / sigmaSquared);
        }

        return PotentialSample{value, gradient};
    }

    RadialPotentialSample getPairPotentialSample(float distance)
    {
        float sigmaOverR = Config::LennardJones::sigma() / distance;
        float sigmaOverR2 = sigmaOverR * sigmaOverR;
        float sigmaOverR6 = sigmaOverR2 * sigmaOverR2 * sigmaOverR2;
        float sigmaOverR12 = sigmaOverR6 * sigmaOverR6;

        float value = 4.0f * Config::LennardJones::EPSILON * (sigmaOverR12 - sigmaOverR6);
        float radialDerivative =
            (24.0f * Config::LennardJones::EPSILON / distance) * (sigmaOverR6 - 2.0f * sigmaOverR12);

        return RadialPotentialSample{value, radialDerivative};
    }

    Vector2 getPairPotentialGradient(const Vector2 &displacement, float distance)
    {
        Vector2 direction = displacement * (1.0f / distance);
        RadialPotentialSample sample = getPairPotentialSample(distance);
        return direction * sample.radialDerivative;
    }
} // namespace

Vector2 getPotentialForce(float x, float y, const Config::Field::ModelConfig &fieldModel)
{
    PotentialSample sample = getSourcesPotentialSample(x, y, fieldModel.sources);
    return sample.gradient * -1.0f;
}

Vector2 computePairForce(const Vector2 &aPosition, const Vector2 &bPosition)
{
    Vector2 displacement = aPosition - bPosition;
    float distanceSquared = displacement.x * displacement.x + displacement.y * displacement.y;

    if (distanceSquared == 0.0f)
    {
        return Vector2(0.0f, 0.0f);
    }

    float distance = std::sqrt(distanceSquared);
    if (distance > Config::LennardJones::cutoff())
    {
        return Vector2(0.0f, 0.0f);
    }

    if (distance < Config::LennardJones::minPairDistance())
    {
        distance = Config::LennardJones::minPairDistance();
    }

    Vector2 gradient = getPairPotentialGradient(displacement, distance);
    return gradient * -1.0f;
}
