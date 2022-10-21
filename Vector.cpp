#include "Vector.h"

// op== between normalized vectors with eps precision
bool eqNormalized(const Vector& v1, const Vector& v2) {
    auto dx = (v1.x - v2.x);
    auto dy = (v1.y - v2.y);
    return dx > -eps && dx<eps&& dy>-eps && dy < eps;
}

// op< between normalized vectors comparing by counterclockwise rotation from (1, 0)
bool ccwLessNormalized(const Vector& v1, const Vector& v2) {
    if (eqNormalized(v1, v2)) return false; // to keep eps-precision
    if (v1.y >= 0 && v2.y < 0) return true;
    if (v1.y < 0 && v2.y >= 0) return false;
    // They are on the same half of the full circle
    return (v1.y >= 0 && v2.y >= 0) != (v1.x < v2.x);
}
