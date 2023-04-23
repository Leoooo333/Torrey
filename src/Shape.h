#pragma once
#include "matrix.h"
#include "parse_scene.h"
#include "transform.h"

//class Object
//{
//public:
//	Shape type;
//	Vector4 ambient;
//	Vector4 diffuse;
//	Vector4 specular;
//	Vector4 emission;
//	//Vector4 transition = Vector4(0.95f);
//	Real shininess;
//	Object(Vector4 ambient, Vector4 diffuse, Vector4 specular, Vector4 emission, Real shininess)
//	{
//		this->ambient = ambient;
//		this->diffuse = diffuse;
//		this->specular = specular;
//		this->emission = emission;
//		this->shininess = shininess;
//	}
//	Vector3 scales;
//	Real FindIntersection(Ray& ray);
//	Matrix4x4 Transform;
//	Matrix4x4 InverseTransform;
//	void SetTransform(Matrix4x4 transform)
//	{
//		this->Transform = transform;
//		this->InverseTransform = inverse(transform);
//	}
//	Vector4 GetNormalByHitPoint(Matrix4x4 hitpoint);
//};

//struct Triangle {
//	int face_index;
//	const ParsedTriangleMesh* mesh;
//};


struct Triangle
{
    const ParsedTriangleMesh* mesh;
    int index;
};

using Shape = std::variant<ParsedSphere, Triangle>;

struct Scene {
    ParsedCamera camera;
    std::vector<ParsedMaterial> materials;
    std::vector<ParsedLight> lights;
    std::vector<Shape> shapes;
    Vector3 background_color;
    int samples_per_pixel;
};





Scene ParsedSceneToScene(ParsedScene& parsed_scene);
Vector4 GetNormalByHitPoint(Vector3 hitpoint, ParsedSphere& sphere);
Vector4 GetNormalByHitPoint(Vector3 hitpoint, Triangle& triangle);

struct Ray
{
    Vector3 Origin;
    Vector3 Direction;
    std::vector<std::shared_ptr<Shape>> Objects;
    std::vector<Real> Distances;
	bool FindIntersection(ParsedSphere& sphere, Matrix4x4 transform, Real t_min, Real t_max)
	{
		Matrix4x4 inverseTransform = inverse(transform);
		Vector4 origin_4 = Vector4(inverseTransform * Vector4(Origin, 1.));
		Vector3 origin = Vector3(origin_4.x, origin_4.y, origin_4.z);
		Vector4 direction_4 = Vector4(inverseTransform * Vector4(Direction, 0.));
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
			Real nearest_t = min(t1, t2);
			if (c / a < 0) // c / a == t1 * t2 < 0 , ray is inside the SphereObj
				nearest_t = t1;
			if (nearest_t >= t_min && nearest_t < t_max)
			{
				Objects.push_back(std::make_shared<Shape>(Shape{ sphere }));
				Distances.push_back(nearest_t);
				return true;
			}
			else
				return false;
		}
	}

	bool FindIntersection(ParsedSphere& sphere, Real t_min, Real t_max)
	{
		return FindIntersection(sphere, translate(Vector3(sphere.position.x, sphere.position.y, sphere.position.z)), t_min, t_max);
	}

	bool FindIntersection(Triangle& triangle, Real t_min, Real t_max)
	{
		Vector3 orig = Origin;
		Vector3 direct = Direction;
		Vector3 p1 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][0]];
		Vector3 p2 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][1]];
		Vector3 p3 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][2]];
		Vector3 normal = normalize(cross((p2 - p1), (p3 - p1)));
		normal = normalize(normal);
		Real nearest_t = dot((p1 - orig), normal) / dot(direct, normal);
		Vector3 p = orig + nearest_t * direct;

		Vector3 c1 = cross(p2 - p1, p - p1);
		Vector3 c2 = cross(p3 - p2, p - p2);
		Vector3 c3 = cross(p1 - p3, p - p3);

		if (dot(c1, c2) >= 0 && dot(c2, c3) >= 0 && dot(c1, c3) >= 0)
		{
			if (nearest_t >= t_min && nearest_t < t_max)
			{
				Objects.push_back(std::make_shared<Shape>(Shape{ triangle }));
				Distances.push_back(nearest_t);
				return true;
			}
			else
				return false;
		}
		else
			return false;
	}

	bool FindIntersection(Shape& shape, Real t_min, Real t_max)
	{
		if (shape.index() == 0) // sphere
			return FindIntersection(std::get<ParsedSphere>(shape), t_min, t_max);
		else
			return FindIntersection(std::get<Triangle>(shape), t_min, t_max);
	}

};


