#pragma once
#include "matrix.h"
#include "parse_scene.h"
#include "transform.h"
#include "Material.h"
#include "3rdparty/pcg.h"

struct Triangle
{
    const ParsedTriangleMesh* mesh;
    int index;
	int area_light_id;
};

using Shape = std::variant<ParsedSphere, Triangle>;

struct Scene {
    ParsedCamera camera;
    std::vector<Material> materials;
    std::vector<ParsedLight> lights;
    std::vector<Shape> shapes;
	std::vector<ParsedShape*> src_shapes;
    Vector3 background_color;
    int samples_per_pixel;
};

class AxisAlignedBoundingBox;

Color ParsedColorToColor(ParsedColor& parsed_color);
Scene ParsedSceneToScene(ParsedScene& parsed_scene);
Vector4 GetNormalByHitPoint(Vector3 hitpoint, ParsedSphere& sphere);
Vector4 GetNormalByHitPoint(Vector3 hitpoint, Triangle& triangle);
Vector4 GetNormalByHitPoint_Smooth(Vector3 hitpoint, Triangle& triangle);
Real GetAreaByShape(ParsedSphere& sphere);
Real GetAreaByShape(Triangle& triangle);


struct Ray
{
    Vector3 Origin;
    Vector3 Direction;
	Real time;
    std::shared_ptr<Shape> object;
    Real distance;
	Real u_coor = -1;
	Real v_coor = -1;
	Vector3 dpdu = {0.,0.,0.};
	Vector3 dpdv = { 0.,0.,0. };
	Real radius = 0.;
	Real spread = 0.;
	bool front_face = true;

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
				Vector3 point_original = (origin + distance * direction) / fabs(sphere.radius);
				Real theta = acos(point_original.y);
				Real phi = atan2(-point_original.z, point_original.x) + c_PI;

	
				u_coor = phi / (2 * c_PI);
				v_coor = theta / c_PI;

				dpdu = sphere.radius * Vector3(sin(v_coor) * sin(u_coor),
					0.,
					sin(v_coor) * cos(u_coor)) * 2. * c_PI;
				dpdv = sphere.radius * Vector3(-cos(v_coor) * cos(u_coor),
					sin(u_coor),
					cos(v_coor) * sin(u_coor)) * c_PI;
				return true;
			}
			else
				return false;
		}
	}

	bool FindIntersection(ParsedSphere& sphere, Real t_min, Real t_max, Scene& scene, pcg32_state& rng)
	{
		Material material = scene.materials[sphere.material_id];
		Vector3 center = { sphere.position.x, sphere.position.y, sphere.position.z };
		if (time != 0) // no motion
			center += time * Vector3{ 0., 0.05, 0. };
		if (material.index() == 7) // volume
		{
			Volume vol = std::get<Volume>(material);
			Real hit_distance = -log(next_pcg32_real<Real>(rng)) * (1. / vol.thickness);
			Real distance_inside_boundary = 0.;
			Ray ray_0 = { Origin, Direction, time };
			if(ray_0.FindIntersection(sphere, translate(center), t_min, t_max))
			{
				Ray ray_1 = { Origin, Direction, time };
				ray_1.Origin += Direction * ray_0.distance;
				ray_1.FindIntersection(sphere, translate(center), t_min, t_max);
				distance_inside_boundary = ray_1.distance;
				if (hit_distance > distance_inside_boundary)
				{
					return false;
				}
				else
				{
					FindIntersection(sphere, translate(center), t_min, t_max);
					distance = ray_0.distance + hit_distance;
					return true;
				}
			}
		}
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

				Real b1 = dot(cross(direct, p3 - p1), orig - p1) /
					dot(cross(direct, p3 - p1), p2 - p1);

				Real b2 = dot(cross(orig - p1, p2 - p1), direct) /
					dot(cross(direct, p3 - p1), p2 - p1);

				Vector3 barycentric = Vector3(1 - b1 - b2, b1, b2);

				if(triangle.mesh->uvs.size() != 0)
				{
					//std::cout << barycentric.x << "," << barycentric.y << "," << barycentric.z << std::endl;
					Vector2 uv_0 = triangle.mesh->uvs[triangle.mesh->indices[triangle.index][0]];
					Vector2 uv_1 = triangle.mesh->uvs[triangle.mesh->indices[triangle.index][1]];
					Vector2 uv_2 = triangle.mesh->uvs[triangle.mesh->indices[triangle.index][2]];


					Vector2 uv_coor = { dot(Vector3(uv_0.x,uv_1.x, uv_2.x),
						barycentric),
					 dot(Vector3(uv_0.y,uv_1.y,uv_2.y),
						barycentric)};
					//here s = b1, t = b2
					Vector2 duvds = uv_2 - uv_0;
					Vector2 duvdt = uv_2 - uv_1;
					Real det = duvds[0] * duvdt[1] - duvdt[0] * duvds[1];
					Real dsdu = duvdt[1] / det;
					Real dtdu = -duvds[1] / det;
					Real dsdv = duvdt[0] / det;
					Real dtdv = -duvds[0] / det;
					Vector3 dpds = p3 - p1;
					Vector3 dpdt = p3 - p2;
					dpdu = dpds * dsdu + dpdt * dtdu;
					dpdv = dpds * dsdv + dpdt * dtdv;

					u_coor = uv_coor.x;
					v_coor = uv_coor.y;
				}
				else
				{
					u_coor = barycentric.y;
					v_coor = barycentric.z;
				}
				if (isnan(u_coor) || isnan(v_coor))
				{
					std::cerr << "u = " << u_coor << ",v = " << v_coor << ")   .\n";
				}
				return true;
			}
			else
				return false;
		}
		else
			return false;
	}

	bool FindIntersection(Triangle& triangle, Real t_min, Real t_max, Scene& scene, pcg32_state& rng)
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

	bool FindIntersection(Shape& shape, Real t_min, Real t_max, Scene& scene, pcg32_state& rng)
	{
		if (shape.index() == 0) // sphere
			return FindIntersection(std::get<ParsedSphere>(shape), t_min, t_max, scene, rng);
		else
			return FindIntersection(std::get<Triangle>(shape), t_min, t_max, scene, rng);
	}

};


