#pragma once
#include "Constants.h"
#include "Point.h"

struct Vector {
    double x;
    double y;
    Vector(double _x, double _y) : x(_x), y(_y) {}
    Vector(Point from, Point to) : x(to.x - from.x), y(to.y - from.y) {}
    double len() const {
        return sqrt(x * x + y * y);
    }
    Vector normalized() const {
        double l = len();
        return Vector(x / l, y / l);
    }
};

// op== between normalized vectors with eps precision
bool eq_normalized(const Vector& v1, const Vector& v2) {
    auto dx = (v1.x - v2.x);
    auto dy = (v1.y - v2.y);
    return dx > -eps && dx<eps&& dy>-eps && dy < eps;
}

// op< between normalized vectors comparing by counterclockwise rotation from (1, 0)
bool ccw_less_normalized(const Vector& v1, const Vector& v2) {
    if (eq_normalized(v1, v2)) return false; // to keep eps-precision
    if (v1.y >= 0 && v2.y < 0) return true;
    if (v1.y < 0 && v2.y >= 0) return false;
    // They are on the same half of the full circle
    return (v1.y >= 0 && v2.y >= 0) != (v1.x < v2.x);
}
