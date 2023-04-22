#pragma once
#include "matrix.h"
#include "parse_scene.h"

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

struct Ray
{
    Vector3 Origin;
    Vector3 Direction;
    std::vector<std::shared_ptr<Shape>> Objects;
    Real t_nearst = 15000.;

};

struct Scene {
    ParsedCamera camera;
    std::vector<ParsedMaterial> materials;
    std::vector<ParsedLight> lights;
    std::vector<Shape> shapes;
    Vector3 background_color;
    int samples_per_pixel;
};

Scene ParsedSceneToScene(ParsedScene& parsed_scene);


bool FindIntersection(Ray& ray, ParsedSphere& sphere, Matrix4x4 transform, Real t_min, Real t_max);
bool FindIntersection(Ray& ray, ParsedSphere& sphere, Real t_min, Real t_max);
bool FindIntersection(Ray& ray, Triangle& triangle, Real t_min, Real t_max);
bool FindIntersection(Ray& ray, Shape& shape, Real t_min, Real t_max);

Vector4 GetNormalByHitPoint(Vector3 hitpoint, ParsedSphere& sphere);
Vector4 GetNormalByHitPoint(Vector3 hitpoint, Triangle& triangle);
