#pragma once
#include "matrix.h"
#include "parse_scene.h"
#include "transform.h"
#include "Material.h"
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
    std::vector<Material> materials;
    std::vector<ParsedLight> lights;
    std::vector<Shape> shapes;
    Vector3 background_color;
    int samples_per_pixel;
};




Color ParsedColorToColor(ParsedColor& parsed_color);
Scene ParsedSceneToScene(ParsedScene& parsed_scene);
Vector4 GetNormalByHitPoint(Vector3 hitpoint, ParsedSphere& sphere);
Vector4 GetNormalByHitPoint(Vector3 hitpoint, Triangle& triangle);

struct Ray
{
    Vector3 Origin;
    Vector3 Direction;
	Real time;
    std::shared_ptr<Shape> object;
    Real distance;
	Real u_coor = -1;
	Real v_coor = -1;

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
				object = std::make_shared<Shape>(Shape{ sphere });
				distance = nearest_t;
				Vector3 point_original = (origin + distance * direction) / sphere.radius;
				Real theta = acos(point_original.y);
				Real phi = atan2(-point_original.z, point_original.x) + c_PI;

				u_coor = phi / (2 * c_PI);
				v_coor = theta / c_PI;
				if (isnan(u_coor))
				{
					std::cerr << "u = " << u_coor << ")   .\n";
				}
				return true;
			}
			else
				return false;
		}
	}

	bool FindIntersection(ParsedSphere& sphere, Real t_min, Real t_max)
	{
		Vector3 center = { sphere.position.x, sphere.position.y, sphere.position.z };
		if (time != 0) // no motion
			center += time * Vector3{ 0., 0.05, 0. };
		return FindIntersection(sphere, translate(center), t_min, t_max);
	}


	bool FindIntersection(Triangle& triangle, Matrix4x4 transform, Real t_min, Real t_max)
	{
		Vector3 orig = Origin;
		if (time != 0) // no motion
			orig += -time * Vector3{ 0., 0.05, 0. };
		Vector3 direct = Direction;
		Vector4 p1_4 = transform* Vector4(triangle.mesh->positions[triangle.mesh->indices[triangle.index][0]], 1.);
		Vector3 p1 = Vector3(p1_4.x, p1_4.y, p1_4.z);
		Vector4 p2_4 = transform * Vector4(triangle.mesh->positions[triangle.mesh->indices[triangle.index][1]], 1.);
		Vector3 p2 = Vector3(p2_4.x, p2_4.y, p2_4.z);
		Vector4 p3_4 = transform * Vector4(triangle.mesh->positions[triangle.mesh->indices[triangle.index][2]], 1.);
		Vector3 p3 = Vector3(p3_4.x, p3_4.y, p3_4.z);


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
				object = std::make_shared<Shape>(Shape{ triangle });
				distance = nearest_t;

				Vector3 s[2];
				for (int i = 2; i--; )
				{
					s[i][0] = p3[i] - p1[i];
					s[i][1] = p2[i] - p1[i];
					s[i][2] = p1[i] - p[i];
				}
				Vector3 b = cross(s[0], s[1]);
				Vector3 barycentric;
				if(b.z == 0.)
				{
					Vector3 ss[2];
					for (int i = 2; i--; )
					{
						ss[i][0] = p3[i+1] - p1[i+1];
						ss[i][1] = p2[i+1] - p1[i+1];
						ss[i][2] = p1[i+1] - p[i+1];
					}
					Vector3 bb = cross(ss[0], ss[1]);
					barycentric = Vector3(1. - (bb.x + bb.y) / bb.z, bb.y / bb.z, bb.x / bb.z);
				}
				else
					barycentric = Vector3(1. - (b.x + b.y) / b.z, b.y / b.z, b.x / b.z);

				if(triangle.mesh->uvs.size() != 0)
				{
					Vector2 uv_coor = 
					{ dot(Vector3(triangle.mesh->uvs[triangle.mesh->indices[triangle.index][0]].x,
						triangle.mesh->uvs[triangle.mesh->indices[triangle.index][1]].x,
						triangle.mesh->uvs[triangle.mesh->indices[triangle.index][2]].x),
						barycentric),
					 dot(Vector3(triangle.mesh->uvs[triangle.mesh->indices[triangle.index][0]].y,
						triangle.mesh->uvs[triangle.mesh->indices[triangle.index][1]].y,
						triangle.mesh->uvs[triangle.mesh->indices[triangle.index][2]].y),
						barycentric)};
					u_coor = uv_coor.x;
					v_coor = uv_coor.y;
				}
				else
				{
					u_coor = barycentric.y;
					v_coor = barycentric.z;
				}
				if (isnan(u_coor))
				{
					std::cerr << "u = " << u_coor << ")   .\n";
				}
				return true;
			}
			else
				return false;
		}
		else
			return false;
	}

	bool FindIntersection(Triangle& triangle, Real t_min, Real t_max)
	{
		Matrix4x4 transform = translate(Vector3(0., 0., 0.));
		return FindIntersection(triangle, transform, t_min, t_max);
	}


	bool FindIntersection(Shape& shape, Matrix4x4 transform, Real t_min, Real t_max)
	{
		if (shape.index() == 0) // sphere
			return FindIntersection(std::get<ParsedSphere>(shape), transform, t_min, t_max);
		else
			return FindIntersection(std::get<Triangle>(shape), transform, t_min, t_max);
	}

	bool FindIntersection(Shape& shape, Real t_min, Real t_max)
	{
		if (shape.index() == 0) // sphere
			return FindIntersection(std::get<ParsedSphere>(shape), t_min, t_max);
		else
			return FindIntersection(std::get<Triangle>(shape), t_min, t_max);
	}

};


