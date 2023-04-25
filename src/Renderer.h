#pragma once

#include <aabb.h>
#include <memory>

#include "Camera.h"
#include "parse_scene.h"
#include "Shape.h"
#include "Variables.h"
#include "3rdparty/pcg.h"
#include <BVH.h>

class Renderer
{
public:
	Renderer() = default;
	void Render(CameraUnion& camera, Variables& vars, Scene& scene,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&));
	void Render_AABB(CameraUnion& camera, Variables& vars, Scene& scene,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&));
	void Render_BVH(CameraUnion& camera, Variables& vars, Scene& scene,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&));
	Real GetLastRendereTime();
	std::shared_ptr<Image3> GetImage();

	Vector3 Miss(const Ray& ray, Scene& scene, int max_depth);
	Vector3 Renderer::Illumination(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Shape& scene);


	Vector3 Illumination_hw_1_1(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_1_2(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_1_3(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_1_4(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_1_5(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_1_6(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_1_7(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_1_8(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_1_9(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_1_10(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_1_11(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_2_1(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_2_2(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_2_3(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_2_4(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_2_5(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);
	Vector3 Illumination_hw_2_6(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars);


	Vector3 Miss_hw_1_1(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_1_2(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_1_3(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_1_4(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_1_5(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_1_6(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_1_7(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_1_8(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_1_9(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_1_10(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_1_11(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_2_1(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_2_2(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_2_3(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_2_4(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_2_5(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_2_6(const Ray& ray, Scene& scene, Variables& vars, int max_depth);

	bool FindNearstIntersection(Ray& ray, Scene& scene, Real t_min, Real t_max);
	bool FindNearstIntersection_AABB(Ray& ray, Scene& scene, std::vector<AxisAlignedBoundingBox>& boxs);
	bool FindNearstIntersection_BVH(Ray& ray, Scene& scene, BVH& node, Real t_min, Real t_max);
private:
	Vector3 TraceRay(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&), pcg32_state& rng);
	Vector3 TraceRay_AABB(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&), pcg32_state& rng,
		std::vector<AxisAlignedBoundingBox>& boxs);
	Vector3 TraceRay_BVH(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&), pcg32_state& rng);
	Vector3 HitNearst(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&), pcg32_state& rng);
	Vector3 HitNearst_BVH(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&), pcg32_state& rng);
	//Vector3 HitNearst_AABB(Ray& ray, Scene& scene, bool isReflect, int max_depth,
	//	Vector3(*miss)(const Ray&, Scene&, Variables&, int),
	//	Vector3(*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&), pcg32_state& rng);
	Vector3 HitPointOriginal(const Ray& ray, Shape& NearstObj);

private:
	Variables m_Vars;
	std::shared_ptr<Image3> m_Image;
	Real m_RenderTime = 0.f;
	BVH m_BVH;
};

bool isFrontFace(const Ray& ray, const Vector3& normal);

Real reflectance(Real cosine, Real ref_idx);

