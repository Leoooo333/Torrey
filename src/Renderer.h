#pragma once

#include <memory>

#include "Camera.h"
#include "parse_scene.h"
#include "Variables.h"

class Renderer
{
public:
	Renderer() = default;
	void Render(CameraUnion& camera, Variables& vars, ParsedScene& scene,
		Vector3(*miss)(const Ray&, ParsedScene&),
		Vector3(*illumination)(Ray&, bool, Vector3, Vector3, ParsedShape&, ParsedScene&));
	Real GetLastRendereTime();
	std::shared_ptr<Image3> GetImage();

	Vector3 Miss(const Ray& ray, ParsedScene& scene);
	Vector3 Renderer::Illumination(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedShape& scene);

private:
	Vector3 TraceRay(Ray& ray, ParsedScene& scene, bool isReflect, int* remain_reflect, int* remain_refract, int refraction_limits,
		Vector3(*miss)(const Ray&, ParsedScene&),
		Vector3(*illumination)(Ray&, bool, Vector3, Vector3, ParsedShape&, ParsedScene&));
	Vector3 HitNearst(Ray& ray, ParsedScene& scene, bool isReflect, int* remain_reflect, int* remain_refract, int refraction_limits,
		Vector3(*miss)(const Ray&, ParsedScene&),
		Vector3(*illumination)(Ray&, bool, Vector3, Vector3, ParsedShape&, ParsedScene&));

	Vector3 HitPointOriginal(const Ray& ray, ParsedShape& NearstObj);
	void FindNearstIntersection(Ray& ray, ParsedScene& scene);

private:
	Real attenuation[3];
	std::shared_ptr<Image3> m_Image;
	Real m_RenderTime = 0.f;
};

Vector3 Illumination_hw_1_1(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene);
Vector3 Illumination_hw_1_2(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene);
Vector3 Miss_hw_1_1(const Ray& ray, ParsedScene& scene);
Vector3 Miss_hw_1_2(const Ray& ray, ParsedScene& scene);