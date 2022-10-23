#pragma once
#include "Vector.h"

double sinOfAngleBetweenNormalizedVectors(const Vector& from, const Vector& to);

double cosOfAngleBetweenNormalizedVectors(const Vector& from, const Vector& to);

Vector rotateNormalizedVectorBy(const Vector& v, double sin_beta, double cos_beta);

struct Rotation {
	double sin_alpha;
	double cos_alpha;
	// how many times we passed the 0 angle.
	unsigned int revolution_num;
};

Rotation rotationOfNormalizedVector(const Vector& v);

Rotation rotationBetweenNormalizedVectors(const Vector& from, const Vector& to);

// The rotation that complements v to 360 degrees
Rotation complementerOfNormalized(const Vector& v);

bool operator<(const Rotation& r1, const Rotation& r2);

Rotation operator+(const Rotation& r1, const Rotation& r2);

Rotation& operator+=(Rotation& r1, const Rotation& r2);

bool operator==(const Rotation&, const Rotation&);

Rotation innerRevolutionOfNGon(unsigned int n);

Rotation outerRevolutionOfNGon(unsigned int n);
