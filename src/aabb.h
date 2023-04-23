#pragma once
#include "vector.h"
#include "Shape.h"

class AxisAlignedBoundingBox
{
public:
	AxisAlignedBoundingBox() {}
	AxisAlignedBoundingBox(const Vector3& min, const Vector3& max) { minimum = min; maximum = max; }
	Vector3 min() const { return minimum; }
	Vector3 max() const { return maximum; }

	Vector3 minimum;
	Vector3 maximum;

    bool isHit(Ray& ray, double t_min, double t_max);
};

AxisAlignedBoundingBox surrounding_box(AxisAlignedBoundingBox& box0, AxisAlignedBoundingBox& box1);


AxisAlignedBoundingBox GetAabbByShape(ParsedSphere& sphere);
AxisAlignedBoundingBox GetAabbByShape(Triangle& triangle);
AxisAlignedBoundingBox GetAabbByShape(Shape& shape);