#include "aabb.h"

AxisAlignedBoundingBox::AxisAlignedBoundingBox(const Vector3& min, const Vector3& max, const Real time)
    : AxisAlignedBoundingBox(min, max)
{
    Vector3 motion = { 0., 0.05, 0. };
    motion *= time;
    AxisAlignedBoundingBox box_after = AxisAlignedBoundingBox(min + motion, max + motion);
    AxisAlignedBoundingBox result = surrounding_box(box_after, *this);
    minimum = result.minimum;
    maximum = result.maximum;
}


bool AxisAlignedBoundingBox::isHit(Ray& ray, double t_min, double t_max)
{
    for (int a = 0; a < 3; a++) {
        auto invD = 1.0 / ray.Direction[a];
        auto t0 = (min()[a] - ray.Origin[a]) * invD;
        auto t1 = (max()[a] - ray.Origin[a]) * invD;
        if (invD < 0.0)
            std::swap(t0, t1);
        t_min = t0 > t_min ? t0 : t_min;
        t_max = t1 < t_max ? t1 : t_max;
        if (t_max < t_min)
            return false;
    }
    return true;
}

AxisAlignedBoundingBox surrounding_box(AxisAlignedBoundingBox& box0, AxisAlignedBoundingBox& box1)
{
    Vector3 small{ min(box0.min().x, box1.min().x),
    min(box0.min().y, box1.min().y),
    min(box0.min().z, box1.min().z) };

    Vector3 big{ max(box0.max().x, box1.max().x),
        max(box0.max().y, box1.max().y),
        max(box0.max().z, box1.max().z) };

    return AxisAlignedBoundingBox(small, big);
}



AxisAlignedBoundingBox GetAabbByShape(ParsedSphere& sphere)
{
	AxisAlignedBoundingBox output_box = AxisAlignedBoundingBox(
		sphere.position - Vector3(fabs(sphere.radius), fabs(sphere.radius), fabs(sphere.radius)),
		sphere.position + Vector3(fabs(sphere.radius), fabs(sphere.radius), fabs(sphere.radius)));
	return output_box;
}
AxisAlignedBoundingBox GetAabbByShape(ParsedSphere& sphere, Real time)
{
    if (time == 0)
        return GetAabbByShape(sphere);
    AxisAlignedBoundingBox output_box = AxisAlignedBoundingBox(
        sphere.position - Vector3(fabs(sphere.radius), fabs(sphere.radius), fabs(sphere.radius)),
        sphere.position + Vector3(fabs(sphere.radius), fabs(sphere.radius), fabs(sphere.radius)), time);
    return output_box;
}

AxisAlignedBoundingBox GetAabbByShape(Triangle& triangle)
{
	Vector3 p0 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][0]];
	Vector3 p1 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][1]];
	Vector3 p2 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][2]];
	AxisAlignedBoundingBox output_box = AxisAlignedBoundingBox(min(p0, p1, p2), max(p0, p1, p2));
	return output_box;
}
AxisAlignedBoundingBox GetAabbByShape(Triangle& triangle, Real time)
{
    if (time == 0)
        return GetAabbByShape(triangle);
    Vector3 p0 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][0]];
    Vector3 p1 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][1]];
    Vector3 p2 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][2]];
    AxisAlignedBoundingBox output_box = AxisAlignedBoundingBox(min(p0, p1, p2), max(p0, p1, p2), time);
    return output_box;
}

AxisAlignedBoundingBox GetAabbByShape(Shape& shape)
{
	if (shape.index() == 0) // sphere
		return GetAabbByShape(std::get<ParsedSphere>(shape));
	else // Triangle
		return GetAabbByShape(std::get<Triangle>(shape));
}

AxisAlignedBoundingBox GetAabbByShape(Shape& shape, Real time)
{
    if (time == 0)
        return GetAabbByShape(shape);
    if (shape.index() == 0) // sphere
        return GetAabbByShape(std::get<ParsedSphere>(shape), time);
    else // Triangle
        return GetAabbByShape(std::get<Triangle>(shape), time);
}

