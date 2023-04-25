#pragma once
#include "vector.h"
#include "Shape.h"

class AxisAlignedBoundingBox
{
public:
	AxisAlignedBoundingBox() {}
	AxisAlignedBoundingBox(const Vector3& min, const Vector3& max) { minimum = min; maximum = max; }
	AxisAlignedBoundingBox(const Vector3& min, const Vector3& max, const Real time);


	Vector3 min() const { return minimum; }
	Vector3 max() const { return maximum; }

	Vector3 minimum;
	Vector3 maximum;

    bool isHit(Ray& ray, double t_min, double t_max);
};

AxisAlignedBoundingBox surrounding_box(AxisAlignedBoundingBox& box0, AxisAlignedBoundingBox& box1);


AxisAlignedBoundingBox GetAabbByShape(ParsedSphere& sphere);
AxisAlignedBoundingBox GetAabbByShape(ParsedSphere& sphere, Real time);

AxisAlignedBoundingBox GetAabbByShape(Triangle& triangle);
AxisAlignedBoundingBox GetAabbByShape(Triangle& triangle, Real time);

AxisAlignedBoundingBox GetAabbByShape(Shape& shape);
AxisAlignedBoundingBox GetAabbByShape(Shape& shape, Real time);
