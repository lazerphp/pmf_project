#ifndef VECTOR2_H
#define VECTOR2_H

struct Vector2
{
    float x, y;

    Vector2(float _x = 0, float _y = 0);

    Vector2 operator+(const Vector2 &other) const;
    Vector2 operator-(const Vector2 &other) const;
    Vector2 operator*(float scalar) const;
    void operator+=(const Vector2 &other);
};

#endif // VECTOR2_H