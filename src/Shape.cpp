#include "Shape.h"
#include "transform.h"

bool FindIntersection(Ray& ray, ParsedSphere& sphere, Matrix4x4 transform, Real t_min, Real t_max)
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
		return false;
	}
	else
	{
		delta = sqrt(delta);
		Real t1 = (-b + delta) / (2 * a);
		Real t2 = (-b - delta) / (2 * a);
		Real t_nearst = min(t1, t2);
		if (c / a < 0) // c / a == t1 * t2 < 0 , ray is inside the SphereObj
			t_nearst = t1;
		if (t_nearst >= t_min && t_nearst < t_max)
		{
			ray.Objects.push_back(std::make_shared<Shape>(Shape{ sphere }));
			ray.t_nearst = t_nearst;
			return true;
		}
		else
			return false;
	}
}

bool FindIntersection(Ray& ray, ParsedSphere& sphere, Real t_min, Real t_max)
{
	return FindIntersection(ray, sphere, translate(Vector3(sphere.position.x, sphere.position.y, sphere.position.z)), t_min, t_max);
}

bool FindIntersection(Ray& ray, Triangle& triangle, Real t_min, Real t_max)
{
	Vector3 origin = ray.Origin;
	Vector3 direction = ray.Direction;
	Vector3 p1 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][0]];
	Vector3 p2 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][1]];
	Vector3 p3 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][2]];
	Vector3 normal = normalize(cross((p2 - p1), (p3 - p1)));
	normal = normalize(normal);
	Real t_nearst = dot((p1 - origin), normal) / dot(direction, normal);
	Vector3 p = origin + t_nearst * direction;

	Vector3 c1 = cross(p2 - p1, p - p1);
	Vector3 c2 = cross(p3 - p2, p - p2);
	Vector3 c3 = cross(p1 - p3, p - p3);

	if (dot(c1, c2) >= 0 && dot(c2, c3) >= 0 && dot(c1, c3) >= 0)
	{
		if (t_nearst >= t_min && t_nearst < t_max)
		{
			ray.Objects.push_back(std::make_shared<Shape>(Shape{ triangle }));
			ray.t_nearst = t_nearst;
			return true;
		}
		else
			return false;
	}
	else
		return false;
}

bool FindIntersection(Ray& ray, Shape& shape, Real t_min, Real t_max)
{
	if (shape.index() == 0) // sphere
		return FindIntersection(ray, std::get<ParsedSphere>(shape), t_min, t_max);
	else
		return FindIntersection(ray, std::get<Triangle>(shape), t_min, t_max);
}

Vector4 GetNormalByHitPoint(Vector3 hitpoint, ParsedSphere& sphere)
{
	Vector4 normal_original = Vector4(hitpoint / sphere.radius, 0.);
	return normal_original;
}
Vector4 GetNormalByHitPoint(Vector3 hitpoint, Triangle& triangle)
{

	Vector3 p1 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][0]];
	Vector3 p2 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][1]];
	Vector3 p3 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][2]];
	Vector3 normal = normalize(cross((p2 - p1), (p3 - p1)));
	normal = normalize(normal);
	return Vector4(normal, 0.);
}



Scene ParsedSceneToScene(ParsedScene& parsed_scene)
{
	std::vector<Shape> shapes;
	for (ParsedShape& shape : parsed_scene.shapes)
	{
		if (shape.index() == 0) // sphere
			shapes.push_back(Shape{ std::get<ParsedSphere>(shape) });
		else // trianglemesh
		{
			ParsedTriangleMesh& mesh = std::get<ParsedTriangleMesh>(shape);
			for (int i = 0; i < mesh.indices.size(); i++)
			{
				shapes.push_back(Shape{ Triangle{&mesh, i} });
			}
		}
	}
	return Scene{ parsed_scene.camera,
	parsed_scene.materials,
	parsed_scene.lights,
	shapes,
	parsed_scene.background_color,
	parsed_scene.samples_per_pixel };
}

