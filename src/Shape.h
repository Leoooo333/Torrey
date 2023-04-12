#pragma once
#include "Ray.h"
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

Real FindIntersection(Ray& ray, ParsedSphere& sphere);
Real FindIntersection(Ray& ray, ParsedTriangleMesh& triangle);

Vector4 GetNormalByHitPoint(Vector3 hitpoint, ParsedSphere& sphere);
Vector4 GetNormalByHitPoint(Vector3 hitpoint, ParsedTriangleMesh& triangle);