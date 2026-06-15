#ifndef VECTOR2_H
#define VECTOR2_H

struct Vector2
{
    float x, y;

    Vector2(float _x = 0, float _y = 0) : x(_x), y(_y) {}

    Vector2 operator+(const Vector2 &other) const
    {
        return Vector2(x + other.x, y + other.y);
    }

    Vector2 operator-(const Vector2 &other) const
    {
        return Vector2(x - other.x, y - other.y);
    }

    Vector2 operator*(float scalar) const
    {
        return Vector2(x * scalar, y * scalar);
    }

    void operator+=(const Vector2 &other)
    {
        x += other.x;
        y += other.y;
    }
};

#endif // VECTOR2_H
