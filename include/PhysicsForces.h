#ifndef PHYSICS_FORCES_H
#define PHYSICS_FORCES_H

#include "Config.h"
#include "Vector2.h"

// Возвращает силу внешнего поля как F = -grad(U).
Vector2 getPotentialForce(float x, float y, const Config::Field::ModelConfig &fieldModel);

// Возвращает силу взаимодействия частицы a со стороны частицы b.
Vector2 computePairForce(const Vector2 &aPosition, const Vector2 &bPosition);

#endif // PHYSICS_FORCES_H
