#pragma once

#include <aabb.h>
#include <memory>

#include "Camera.h"
#include "parse_scene.h"
#include "Shape.h"
#include "Variables.h"
#include "3rdparty/pcg.h"
#include <BVH.h>

class Sampler
{
public:
	virtual Vector3 GetBRDF(Material material, Ray ray, Vector3 normal, Variables& m_Vars, const Vector3 direction, Scene& scene, BVH& node, pcg32_state& rng, bool is_reflect, bool is_refract) { return Vector3(0., 0., 0.); }
	virtual Real GetPDF(Material material, Ray ray, Vector3 normal, Variables& m_Vars, const Vector3 direction, Scene& scene, BVH& node, pcg32_state& rng, bool is_reflect, bool is_refract) { return 0.; }
	virtual Vector3 SampleDirection(Material material, Ray ray, Vector3 normal, Variables& m_Vars, Scene& scene, BVH& node, pcg32_state& rng, bool* is_reflect, bool* is_refract) { return Vector3(0., 1., 0.); }
	virtual Vector3 GetTvalue(Material material, Ray ray, Vector3 normal, Variables& m_Vars, const Vector3 direction, Scene& scene, BVH& node, pcg32_state& rng) { return Vector3(0., 0., 0.); }
};

struct MIS_Infor
{
	Vector3 L_value;
	Vector3 T_value_next;
	Real PDF_value;
};

class Renderer
{
public:
	Renderer() = default;
	void Render(CameraUnion& camera, Variables& vars, Scene& scene,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&));
	void Render_AABB(CameraUnion& camera, Variables& vars, Scene& scene,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&));
	void Render_BVH(CameraUnion& camera, Variables& vars, Scene& scene,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&));
	void Render_BVH_Path(CameraUnion& camera, Variables& vars, Scene& scene,
		Vector3(Renderer::* miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::* illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&));
	//One_Sample
	void Render_BVH_Path_One_Sample(CameraUnion& camera, Variables& vars, Scene& scene,
		Vector3(Renderer::* miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::* illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&));


	Real GetLastRendereTime();
	std::shared_ptr<Image3> GetImage();

	Vector3 Miss(const Ray& ray, Scene& scene, int max_depth);
	// return (color, surface_color)
	MIS_Infor Renderer::Light_Sampling(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	MIS_Infor Renderer::BRDF_Sampling(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);

	Vector3 Illumination_hw_1_1(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_1_2(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_1_3(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_1_4(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_1_5(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_1_6(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_1_7(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_1_8(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_1_9(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_1_10(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_1_11(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_2_1(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_2_2(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_2_3(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_2_4(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_2_5(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_3_1(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_3_3(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_3_4(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_3_7(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_3_9(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_3_11(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_4_1(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_4_2(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_4_3(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);
	Vector3 Illumination_hw_4_4(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
		Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng);

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
	Vector3 Miss_hw_3_1(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_3_3(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_3_4(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_3_7(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_3_9(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_3_11(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_4_1(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_4_2(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_4_3(const Ray& ray, Scene& scene, Variables& vars, int max_depth);
	Vector3 Miss_hw_4_4(const Ray& ray, Scene& scene, Variables& vars, int max_depth);


	bool FindNearstIntersection(Ray& ray, Scene& scene, Real t_min, Real t_max, pcg32_state& rng);
	bool FindNearstIntersection_AABB(Ray& ray, Scene& scene, std::vector<AxisAlignedBoundingBox>& boxs);
	static bool FindNearstIntersection_BVH(Ray& ray, Scene& scene, BVH& node, Real t_min, Real t_max, pcg32_state& rng);
private:
	Vector3 TraceRay(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng);
	Vector3 TraceRay_AABB(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng,
		std::vector<AxisAlignedBoundingBox>& boxs);
	Vector3 TraceRay_BVH(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng);
	Vector3 TraceRay_BVH_Path(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::* miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::* illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng);
	//One_Sample
	Vector3 TraceRay_BVH_Path_One_Sample(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::* miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::* illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng);

	Vector3 HitNearst(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng);
	Vector3 HitNearst_BVH(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng);
	Vector3 HitNearst_BVH_Path(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::* miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::* illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng);
	//One_Sample
	Vector3 HitNearst_BVH_Path_One_Sample(Ray& ray, Scene& scene, bool isReflect, int max_depth,
		Vector3(Renderer::* miss)(const Ray&, Scene&, Variables&, int),
		Vector3(Renderer::* illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng);

	//Vector3 HitNearst_AABB(Ray& ray, Scene& scene, bool isReflect, int max_depth,
	//	Vector3(*miss)(const Ray&, Scene&, Variables&, int),
	//	Vector3(*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&), pcg32_state& rng);
	Vector3 HitPointOriginal(const Ray& ray, Shape& NearstObj);

private:
	Variables m_Vars;
	std::shared_ptr<Image3> m_Image;
	Real m_RenderTime = 0.f;
public:
	BVH m_BVH;
	Sampler* m_Sampler=nullptr;
};




bool isFrontFace(const Ray& ray, const Vector3& normal);

Real reflectance(Real cosine, Real ref_idx);

inline Real GetGeoConfig(const Real theta, const Real alpha)
{
	Real a = sqrt(alpha / 2. + 1.) / tan(theta);
	Real G_w;
	if (a < 1.6)
	{
		G_w = a * (3.535 + 2.181 * a) / (1. + a * (2.276 + 2.577 * a));
	}
	else
		G_w = 1;
	return G_w;
}
