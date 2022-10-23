#include "Trig.h"

double sinOfAngleBetweenNormalizedVectors(const Vector& from, const Vector& to) {
	// Cross product's Z dimension has length |from||to|sin(alpha) == sin(alpha)
	return from.x * to.y - from.y * to.x;
}

double cosOfAngleBetweenNormalizedVectors(const Vector& from, const Vector& to) {
	// Dot product is |from||to|cos(alpha) == cos(alpha)
	return from.x * to.x + from.y * to.y;
}

Vector rotateNormalizedVectorBy(const Vector& v, double sin_beta, double cos_beta) {
	double cos_alpha = v.x;
	double sin_alpha = v.y;
	return Vector(
		// x = cos(alpha+beta) == cos(alpha)cos(beta)-sin(alpha)sin(beta)
		cos_alpha * cos_beta - sin_alpha * sin_beta,
		// y = sin(alpha+beta) == sin(alpha)cos(beta)+cos(alpha)sin(beta)
		sin_alpha * cos_beta + cos_alpha * sin_beta
	).normalized();
}

Rotation rotationOfNormalizedVector(const Vector& v) {
	return Rotation(v.y, v.x, 0);
}

Rotation rotationBetweenNormalizedVectors(const Vector& from, const Vector& to) {
	return Rotation(
		sinOfAngleBetweenNormalizedVectors(from, to),
		cosOfAngleBetweenNormalizedVectors(from, to),
		0);
}

// The rotation that complements v to 360 degrees
Rotation complementerOfNormalized(const Vector& v) {
	static constexpr Vector east(1.0, 0.0);
	return Rotation(
		sinOfAngleBetweenNormalizedVectors(v, east),
		cosOfAngleBetweenNormalizedVectors(v, east),
		0
	);
}

bool operator<(const Rotation& r1, const Rotation& r2) {
	return r1.revolution_num < r2.revolution_num ||
		(r1.revolution_num == r2.revolution_num &&
			ccwLessNormalized(Vector(r1.cos_alpha, r1.sin_alpha),
				Vector(r2.cos_alpha, r2.sin_alpha))
			);
}

Rotation operator+(const Rotation& r1, const Rotation& r2) {
	Vector v1 = Vector(r1.cos_alpha, r2.sin_alpha);
	auto rotated_vector = rotateNormalizedVectorBy(
		v1, r2.sin_alpha, r2.cos_alpha);
	Rotation ret{
		rotated_vector.y, rotated_vector.x,
		r1.revolution_num + r2.revolution_num
	};
	if (!(r2 < complementerOfNormalized(v1))) {
		++ret.revolution_num;
	}
	return ret;
}

Rotation& operator+=(Rotation& r1, const Rotation& r2) {
	r1 = r1 + r2;
	return r1;
}

bool operator==(const Rotation& one, const Rotation& other) {
	return (one.revolution_num == other.revolution_num &&
		abs(one.sin_alpha - other.sin_alpha) < eps &&
		abs(one.cos_alpha - other.cos_alpha) < eps) ||
		(
			(
				(one.revolution_num > other.revolution_num &&
				 one.revolution_num - other.revolution_num == 1) ||
				(other.revolution_num > one.revolution_num &&
				 other.revolution_num - one.revolution_num == 1)
			) &&
			abs(one.sin_alpha) < eps &&
			abs(other.sin_alpha) < eps &&
			abs(one.cos_alpha - 1.0) < eps &&
			abs(other.cos_alpha - 1.0) < eps
		);
}

Rotation innerRevolutionOfNGon(unsigned int n) {
	return Rotation{
		0.0,
		n%2 == 0 ? 1.0 : -1.0,
		(n - 2) / 2
	};
}

Rotation outerRevolutionOfNGon(unsigned int n) {
	return Rotation{
		0.0,
		n % 2 == 0 ? 1.0 : -1.0,
		(n + 1) / 2
	};
}
