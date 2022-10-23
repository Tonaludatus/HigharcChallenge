#pragma once
#include <cmath>
#include "Constants.h"
#include "Point.h"

struct Vector {
    double x;
    double y;
    constexpr Vector(double _x, double _y) : x(_x), y(_y) {}
    Vector(Point from, Point to) : x(to.x - from.x), y(to.y - from.y) {}
    double len() const {
        return sqrt(x * x + y * y);
    }
    Vector normalized() const {
        double l = len();
        return Vector(x / l, y / l);
    }
    Vector flipped() const {
        return Vector(-x, -y);
    }
    Point asPoint() const {
        return Point(x, y);
    }
    Point translate(Point p) const {
        return Point(p.x + x, p.y + y);
    }
};

// op== between normalized vectors with eps precision
bool eqNormalized(const Vector& v1, const Vector& v2);

// op< between normalized vectors comparing by counterclockwise rotation from (1, 0)
bool ccwLessNormalized(const Vector& v1, const Vector& v2);
