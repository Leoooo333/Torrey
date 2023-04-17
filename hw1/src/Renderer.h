#pragma once

#include <memory>

#include "Camera.h"
#include "parse_scene.h"
#include "Variables.h"
#include "3rdparty/pcg.h"

class Renderer
{
public:
	Renderer() = default;
	void Render(CameraUnion& camera, Variables& vars, ParsedScene& scene,
		Vector3(*miss)(const Ray&, ParsedScene&, Variables&, int),
		Vector3(*illumination)(Ray&, bool, Vector3, Vector3, ParsedShape&, ParsedScene&, Variables&));
	Real GetLastRendereTime();
	std::shared_ptr<Image3> GetImage();

	Vector3 Miss(const Ray& ray, ParsedScene& scene, int max_depth);
	Vector3 Renderer::Illumination(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedShape& scene);

	static void FindNearstIntersection(Ray& ray, ParsedScene& scene);
private:
	Vector3 TraceRay(Ray& ray, ParsedScene& scene, bool isReflect, int max_depth,
		Vector3(*miss)(const Ray&, ParsedScene&, Variables&, int),
		Vector3(*illumination)(Ray&, bool, Vector3, Vector3, ParsedShape&, ParsedScene&, Variables&), pcg32_state& rng);
	Vector3 HitNearst(Ray& ray, ParsedScene& scene, bool isReflect, int max_depth,
		Vector3(*miss)(const Ray&, ParsedScene&, Variables&, int),
		Vector3(*illumination)(Ray&, bool, Vector3, Vector3, ParsedShape&, ParsedScene&, Variables&), pcg32_state& rng);
	
	Vector3 HitPointOriginal(const Ray& ray, ParsedShape& NearstObj);


private:
	Variables m_Vars;
	std::shared_ptr<Image3> m_Image;
	Real m_RenderTime = 0.f;
};

bool isFrontFace(const Ray& ray, const Vector3& normal);

Real reflectance(Real cosine, Real ref_idx);

Vector3 Illumination_hw_1_1(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars);
Vector3 Illumination_hw_1_2(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars);
Vector3 Illumination_hw_1_3(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars);
Vector3 Illumination_hw_1_4(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars);
Vector3 Illumination_hw_1_5(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars);
Vector3 Illumination_hw_1_6(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars);
Vector3 Illumination_hw_1_7(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars);
Vector3 Illumination_hw_1_8(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars);
Vector3 Illumination_hw_1_9(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars);
Vector3 Illumination_hw_1_10(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars);
Vector3 Illumination_hw_1_11(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars);
																			 
Vector3 Miss_hw_1_1(const Ray& ray, ParsedScene& scene, Variables& vars, int max_depth);
Vector3 Miss_hw_1_2(const Ray& ray, ParsedScene& scene, Variables& vars, int max_depth);
Vector3 Miss_hw_1_3(const Ray& ray, ParsedScene& scene, Variables& vars, int max_depth);
Vector3 Miss_hw_1_4(const Ray& ray, ParsedScene& scene, Variables& vars, int max_depth);
Vector3 Miss_hw_1_5(const Ray& ray, ParsedScene& scene, Variables& vars, int max_depth);
Vector3 Miss_hw_1_6(const Ray& ray, ParsedScene& scene, Variables& vars, int max_depth);
Vector3 Miss_hw_1_7(const Ray& ray, ParsedScene& scene, Variables& vars, int max_depth);
Vector3 Miss_hw_1_8(const Ray& ray, ParsedScene& scene, Variables& vars, int max_depth);
Vector3 Miss_hw_1_9(const Ray& ray, ParsedScene& scene, Variables& vars, int max_depth);
Vector3 Miss_hw_1_10(const Ray& ray, ParsedScene& scene, Variables& vars, int max_depth);
Vector3 Miss_hw_1_11(const Ray& ray, ParsedScene& scene, Variables& vars, int max_depth);