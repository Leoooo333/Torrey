#include "Shape.h"
#include "transform.h"

Real FindIntersection(Ray& ray, ParsedSphere& sphere, Matrix4x4 transform)
{
	Matrix4x4 inverseTransform = inverse(transform);
	Vector4 origin_4 = Vector4(inverseTransform * Vector4(ray.Origin, 1.));
	Vector3 origin = Vector3(origin_4.x, origin_4.y, origin_4.z);
	Vector4 direction_4 = Vector4(inverseTransform * Vector4(ray.Direction, 0.));
	Vector3 direction = Vector3(direction_4.x, direction_4.y, direction_4.z);

	Real c = dot(origin, origin) - sphere.radius * sphere.radius;
	Real b = 2. * dot(origin, direction);
	Real a = dot(direction, direction);

	Real delta = b * b - 4 * a * c;
	if (delta < 0)
	{
		return -1.;
	}
	else
	{
		delta = sqrt(delta);
		Real t1 = (-b + delta) / (2 * a);
		Real t2 = (-b - delta) / (2 * a);
		Real t_nearst = min(t1, t2);
		if (c / a < 0) // c / a == t1 * t2 < 0 , ray is inside the SphereObj
			t_nearst = t1;
		return t_nearst;
	}
}

Real FindIntersection(Ray& ray, ParsedSphere& sphere)
{
	return FindIntersection(ray, sphere, translate(Vector3(sphere.position.x, sphere.position.y, sphere.position.z)));
}

//Real FindIntersection(Ray& ray, ParsedTriangleMesh& triangle)
//{
//	Vector3 origin = ray.Origin;
//	Vector3 direction = ray.Direction;
//	Vector3 normal = normalize(cross((p2 - p1), (p3 - p1)));
//	normal = normalize(normal);
//	float t_nearst = glm::dot((p1 - origin), normal) / glm::dot(direction, normal);
//	glm::vec3 p = origin + t_nearst * direction;
//
//	vec3 c1 = cross(p2 - p1, p - p1);
//	vec3 c2 = cross(p3 - p2, p - p2);
//	vec3 c3 = cross(p1 - p3, p - p3);
//
//	if (dot(c1, c2) >= 0 && dot(c2, c3) >= 0 && dot(c1, c3) >= 0)
//	{
//		return t_nearst;
//	}
//	else
//		return -1.f;
//}

Vector4 GetNormalByHitPoint(Vector3 hitpoint, ParsedSphere& sphere)
{
	Vector4 normal_original = Vector4(hitpoint, 0.);
	return normal_original;
}
//Vector4 GetNormalByHitPoint(Vector4 hitpoint, ParsedTriangleMesh& triangle)
//{
//	glm::vec3 normal = normalize(cross((p2 - p1), (p3 - p1)));
//	//normal = vec3(transpose(this->InverseTransform) * vec4(normal, 0.f));
//	normal = normalize(normal);
//	return vec4(normal, 0.f);
//}
