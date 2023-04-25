//#pragma comment(lib, "FreeImage.lib")
#include "Renderer.h"

#include "parallel.h"
#include "3rdparty\pcg.h"
#include "Shape.h"
#include "timer.h"
#include "transform.h"
#include "progressreporter.h"

#define TILE_SIZE 64
void Renderer::Render(CameraUnion& camera, Variables& vars, Scene& scene,
                      Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
                      Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&))
{
	m_Vars = vars;

	int width = std::visit([=](auto& cam) {return cam.m_CameraParameters.width; }, camera);
	int height = std::visit([=](auto& cam) {return cam.m_CameraParameters.height; }, camera);
	int samples_per_pixel = std::visit([=](auto& cam) {return cam.m_CameraParameters.samples_per_pixel; }, camera);
	m_Image = std::make_shared<Image3>(width, height);

	Timer start_time;
	tick(start_time);

	if (samples_per_pixel == 1)
	{
		const int tile_size = vars.tile_size;
		int num_tiles_x = (width + tile_size - 1) / tile_size;
		int num_tiles_y = (height + tile_size - 1) / tile_size;
		ProgressReporter reporter(num_tiles_y * num_tiles_x);


		parallel_for([&](const Vector2i& tile) {
			pcg32_state rng = init_pcg32(tile[1] * num_tiles_x + tile[0] /* seed */);
			int x0 = tile[0] * tile_size;
			int x1 = min(x0 + tile_size, width);
			int y0 = tile[1] * tile_size;
			int y1 = min(y0 + tile_size, height);
			for (int y = y0; y < y1; y++)
			{
				for (int x = x0; x < x1; x++)
				{
					Vector3 color = Vector3(0., 0., 0.);
					Vector2 offset = Vector2(0.5, 0.5);
					Ray ray = std::visit([=](auto& cam) {return cam.CalculateRayDirections(x, y, offset, rng); }, camera);

					color = TraceRay(ray, scene, true, vars.max_depth, miss, illumination, rng);

					(*m_Image)(x, height - 1 - y) = color;

				}
			}
			reporter.update(1);

			}, Vector2i(num_tiles_x, num_tiles_y));
		m_RenderTime = tick(start_time);
		std::cout << "Total Time:	" << m_RenderTime << std::endl;
	}
	else if(samples_per_pixel > 1)
	{
		const int tile_size = vars.tile_size;
		int num_tiles_x = (width + tile_size - 1) / tile_size;
		int num_tiles_y = (height + tile_size - 1) / tile_size;
		ProgressReporter reporter(num_tiles_y * num_tiles_x);

		parallel_for([&](const Vector2i& tile) {
			pcg32_state rng = init_pcg32(tile[1] * num_tiles_x + tile[0] /* seed */);
			int x0 = tile[0] * tile_size;
			int x1 = min(x0 + tile_size, width);
			int y0 = tile[1] * tile_size;
			int y1 = min(y0 + tile_size, height);
				
			for (int y = y0; y < y1; y++)
			{
				for (int x = x0; x < x1; x++)
				{
					Vector3 color = Vector3(0., 0., 0.);
					for (int s = 0; s < samples_per_pixel; s++)
					{
						Vector2 offset = { next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng) };
						Ray ray = std::visit([=](auto& cam) {return cam.CalculateRayDirections(x, y, offset, rng); }, camera);

						color += TraceRay(ray, scene, true, vars.max_depth, miss, illumination, rng);

					}
					color /= (Real)samples_per_pixel;
					(*m_Image)(x, height - 1 - y) = color;

					//m_Vars.isPrimary_ray = true;
				}
			}
			reporter.update(1);
			}, Vector2i(num_tiles_x, num_tiles_y));
		m_RenderTime = tick(start_time);
		std::cout << "Total Time:	" << m_RenderTime << std::endl;
	}
}

void Renderer::Render_AABB(CameraUnion& camera, Variables& vars, Scene& scene,
	Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
	Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&))
{
	m_Vars = vars;

	int width = std::visit([=](auto& cam) {return cam.m_CameraParameters.width; }, camera);
	int height = std::visit([=](auto& cam) {return cam.m_CameraParameters.height; }, camera);
	int samples_per_pixel = std::visit([=](auto& cam) {return cam.m_CameraParameters.samples_per_pixel; }, camera);
	m_Image = std::make_shared<Image3>(width, height);

	Timer start_time;
	tick(start_time);

	std::vector<AxisAlignedBoundingBox> boxs;
	for(Shape shape : scene.shapes)
	{
		if (shape.index() == 0) // sphere
			boxs.push_back(GetAabbByShape(std::get<ParsedSphere>(shape)));
		else // trianglemesh
		{
			boxs.push_back(GetAabbByShape(std::get<Triangle>(shape)));
		}
	}

	if (samples_per_pixel == 1)
	{
		const int tile_size = vars.tile_size;
		int num_tiles_x = (width + tile_size - 1) / tile_size;
		int num_tiles_y = (height + tile_size - 1) / tile_size;
		ProgressReporter reporter(num_tiles_y * num_tiles_x);

		parallel_for([&](const Vector2i& tile) {
			pcg32_state rng = init_pcg32(tile[1] * num_tiles_x + tile[0] /* seed */);
			int x0 = tile[0] * tile_size;
			int x1 = min(x0 + tile_size, width);
			int y0 = tile[1] * tile_size;
			int y1 = min(y0 + tile_size, height);
			for (int y = y0; y < y1; y++)
			{
				for (int x = x0; x < x1; x++)
				{
					Vector3 color = Vector3(0., 0., 0.);
					Vector2 offset = Vector2(0.5, 0.5);
					Ray ray = std::visit([=](auto& cam) {return cam.CalculateRayDirections(x, y, offset, rng); }, camera);

					color = TraceRay_AABB(ray, scene, true, vars.max_depth, miss, illumination, rng, boxs);

					(*m_Image)(x, height - 1 - y) = color;

				}
			}
			reporter.update(1);
			}, Vector2i(num_tiles_x, num_tiles_y));
		m_RenderTime = tick(start_time);
		std::cout << "Total Time:	" << m_RenderTime << std::endl;
	}
	else if (samples_per_pixel > 1)
	{
		const int tile_size = vars.tile_size;
		int num_tiles_x = (width + tile_size - 1) / tile_size;
		int num_tiles_y = (height + tile_size - 1) / tile_size;
		ProgressReporter reporter(num_tiles_y* num_tiles_x);

		parallel_for([&](const Vector2i& tile) {
			pcg32_state rng = init_pcg32(tile[1] * num_tiles_x + tile[0] /* seed */);
			int x0 = tile[0] * tile_size;
			int x1 = min(x0 + tile_size, width);
			int y0 = tile[1] * tile_size;
			int y1 = min(y0 + tile_size, height);
			for (int y = y0; y < y1; y++)
			{
				for (int x = x0; x < x1; x++)
				{
					Vector3 color = Vector3(0., 0., 0.);
					for (int s = 0; s < samples_per_pixel; s++)
					{
						Vector2 offset = { next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng) };
						Ray ray = std::visit([=](auto& cam) {return cam.CalculateRayDirections(x, y, offset, rng); }, camera);

						color += TraceRay_AABB(ray, scene, true, vars.max_depth, miss, illumination, rng, boxs);

					}
					color /= (Real)samples_per_pixel;
					(*m_Image)(x, height - 1 - y) = color;

					//m_Vars.isPrimary_ray = true;
				}
			
			}
			reporter.update(1);
			}, Vector2i(num_tiles_x, num_tiles_y));
		m_RenderTime = tick(start_time);
		std::cout << "Total Time:	" << m_RenderTime << std::endl;
	}
}

void Renderer::Render_BVH(CameraUnion& camera, Variables& vars, Scene& scene,
	Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
	Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&))
{
	m_Vars = vars;

	int width = std::visit([=](auto& cam) {return cam.m_CameraParameters.width; }, camera);
	int height = std::visit([=](auto& cam) {return cam.m_CameraParameters.height; }, camera);
	int samples_per_pixel = std::visit([=](auto& cam) {return cam.m_CameraParameters.samples_per_pixel; }, camera);
	m_Image = std::make_shared<Image3>(width, height);

	Timer start_time;
	tick(start_time);

	//std::vector<AxisAlignedBoundingBox> boxs;
	//for (Shape shape : scene.shapes)
	//{
	//	if (shape.index() == 0) // sphere
	//		boxs.push_back(GetAabbByShape(std::get<ParsedSphere>(shape)));
	//	else // trianglemesh
	//	{
	//		boxs.push_back(GetAabbByShape(std::get<Triangle>(shape)));
	//	}
	//}
	std::vector<std::shared_ptr<Shape>> src_shapes;
	for(Shape& shape : scene.shapes)
	{
		src_shapes.push_back(std::make_shared<Shape>(shape));
	}
	pcg32_state rng_bvh = init_pcg32();

	Timer build_time;
	tick(build_time);


	if(m_Vars.parallel_counts_bvh > 1)
	{
		m_BVH = BVH(src_shapes, 0, 0, rng_bvh, m_Vars.parallel_counts_bvh, m_Vars.tile_size_bvh);
	}
	else 	// without parallel build
		m_BVH = BVH(src_shapes, 0, 0, rng_bvh);
	std::cout << "Build BVH Time:	" << tick(build_time) << std::endl;
	if (samples_per_pixel == 1)
	{
		const int tile_size = vars.tile_size;
		int num_tiles_x = (width + tile_size - 1) / tile_size;
		int num_tiles_y = (height + tile_size - 1) / tile_size;
		ProgressReporter reporter(num_tiles_y * num_tiles_x);

		if(m_Vars.motion_blur_samples == 0)
			parallel_for([&](const Vector2i& tile) {
				pcg32_state rng = init_pcg32(tile[1] * num_tiles_x + tile[0] /* seed */);
				int x0 = tile[0] * tile_size;
				int x1 = min(x0 + tile_size, width);
				int y0 = tile[1] * tile_size;
				int y1 = min(y0 + tile_size, height);
				for (int y = y0; y < y1; y++)
				{
					for (int x = x0; x < x1; x++)
					{
						Vector3 color = Vector3(0., 0., 0.);
						Vector2 offset = Vector2(0.5, 0.5);	
						Ray ray = std::visit([=](auto& cam) {return cam.CalculateRayDirections(x, y, offset, rng); }, camera);

						color = TraceRay_BVH(ray, scene, true, vars.max_depth, miss, illumination, rng);

						(*m_Image)(x, height - 1 - y) = color;

					}
				}
				reporter.update(1);
				}, Vector2i(num_tiles_x, num_tiles_y));
		else
			parallel_for([&](const Vector2i& tile) {
			pcg32_state rng = init_pcg32(tile[1] * num_tiles_x + tile[0] /* seed */);
			int x0 = tile[0] * tile_size;
			int x1 = min(x0 + tile_size, width);
			int y0 = tile[1] * tile_size;
			int y1 = min(y0 + tile_size, height);
			for (int y = y0; y < y1; y++)
			{
				for (int x = x0; x < x1; x++)
				{
					Vector3 color = Vector3(0., 0., 0.);
					for (int t = 0; t < m_Vars.motion_blur_samples; t++)
					{
						Vector2 offset = Vector2(0.5, 0.5);
						Real time_now = next_pcg32_real<Real>(rng);
						Ray ray = std::visit([=](auto& cam) {return cam.CalculateRayDirections(x, y, offset, rng, time_now); }, camera);

						color += TraceRay_BVH(ray, scene, true, vars.max_depth, miss, illumination, rng);
					}


					(*m_Image)(x, height - 1 - y) = color / (Real)m_Vars.motion_blur_samples;

				}
			}
			reporter.update(1);
				}, Vector2i(num_tiles_x, num_tiles_y));
		m_RenderTime = tick(start_time);
		std::cout << "Total Time:	" << m_RenderTime << std::endl;
	}
	else if (samples_per_pixel > 1)
	{
		const int tile_size = vars.tile_size;
		int num_tiles_x = (width + tile_size - 1) / tile_size;
		int num_tiles_y = (height + tile_size - 1) / tile_size;
		ProgressReporter reporter(num_tiles_y * num_tiles_x);

		if (m_Vars.motion_blur_samples == 0)
			parallel_for([&](const Vector2i& tile) {
				pcg32_state rng = init_pcg32(tile[1] * num_tiles_x + tile[0] /* seed */);
				int x0 = tile[0] * tile_size;
				int x1 = min(x0 + tile_size, width);
				int y0 = tile[1] * tile_size;
				int y1 = min(y0 + tile_size, height);
				for (int y = y0; y < y1; y++)
				{
					for (int x = x0; x < x1; x++)
					{
						Vector3 color = Vector3(0., 0., 0.);
						for (int s = 0; s < samples_per_pixel; s++)
						{
							Vector2 offset = { next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng) };
							Ray ray = std::visit([=](auto& cam) {return cam.CalculateRayDirections(x, y, offset, rng); }, camera);
							color += TraceRay_BVH(ray, scene, true, vars.max_depth, miss, illumination, rng);

						}
						color /= (Real)samples_per_pixel;
						(*m_Image)(x, height - 1 - y) = color;

						//m_Vars.isPrimary_ray = true;
					}
					//if (y % 1 == 0)
					//{
					//	std::cout << "Finish:	" << (Real)y / (Real)height << std::endl;
					//}
				}
				reporter.update(1);
				}, Vector2i(num_tiles_x, num_tiles_y));
		else
			parallel_for([&](const Vector2i& tile) {
			pcg32_state rng = init_pcg32(tile[1] * num_tiles_x + tile[0] /* seed */);
			int x0 = tile[0] * tile_size;
			int x1 = min(x0 + tile_size, width);
			int y0 = tile[1] * tile_size;
			int y1 = min(y0 + tile_size, height);
			for (int y = y0; y < y1; y++)
			{
				for (int x = x0; x < x1; x++)
				{
					Vector3 color = Vector3(0., 0., 0.);
					for (int s = 0; s < samples_per_pixel; s++)
					{
						Vector2 offset = { next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng) };
						for (int t = 0; t < m_Vars.motion_blur_samples; t++)
						{
							Real time_now = next_pcg32_real<Real>(rng);
							Ray ray = std::visit([=](auto& cam) {return cam.CalculateRayDirections(x, y, offset, rng, time_now); }, camera);
							color += TraceRay_BVH(ray, scene, true, vars.max_depth, miss, illumination, rng);
						}
						color /= (Real)m_Vars.motion_blur_samples;
					}
					color /= (Real)samples_per_pixel;
					(*m_Image)(x, height - 1 - y) = color;

					//m_Vars.isPrimary_ray = true;
				}
				//if (y % 1 == 0)
				//{
				//	std::cout << "Finish:	" << (Real)y / (Real)height << std::endl;
				//}
			}
			reporter.update(1);
				}, Vector2i(num_tiles_x, num_tiles_y));
		m_RenderTime = tick(start_time);
		std::cout << "Total Time:	" << m_RenderTime << std::endl;
	}
}


Real Renderer::GetLastRendereTime()
{
	return m_RenderTime;
}

std::shared_ptr<Image3> Renderer::GetImage()
{
	return m_Image;
}

Vector3 Renderer::TraceRay(Ray& ray, Scene& scene, bool isReflect, int max_depth,
	Vector3 (Renderer::*miss)(const Ray&, Scene&, Variables&, int),
	Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&), pcg32_state& rng)
{
	if (!FindNearstIntersection(ray, scene, m_Vars.t_min, m_Vars.t_max)) // Miss
	{
		return (this->*miss)(ray, scene, m_Vars, max_depth);
	}
	else
	{
		if (isReflect)
			return HitNearst(ray, scene, true, max_depth, miss, illumination, rng);
		else
			return HitNearst(ray, scene, false, max_depth, miss, illumination, rng);
	}
}

Vector3 Renderer::TraceRay_AABB(Ray& ray, Scene& scene, bool isReflect, int max_depth,
	Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
	Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&), pcg32_state& rng,
	std::vector<AxisAlignedBoundingBox>& boxs)
{
	//Shape& NearstObj = scene.shapes[ray.Objects];
	//AxisAlignedBoundingBox aabb;
	//if (NearstObj.index() == 0) // is sphere
	//{
	//	ParsedSphere& sphere = std::get<ParsedSphere>(NearstObj);
	//	aabb = GetAabbByShape(sphere);
	//}
	//else //is triangle
	//{
	//	ParsedTriangleMesh& triangle = std::get<ParsedTriangleMesh>(NearstObj);
	//	aabb = GetAabbByShape(triangle, ray.NearstIndexinMesh);
	//}

	
	if (FindNearstIntersection_AABB(ray, scene, boxs)) // Miss
	{
		return Vector3(1., 1., 1.);
	}
	else
	{
		return (this->*miss)(ray, scene, m_Vars, max_depth);
	}
}

Vector3 Renderer::TraceRay_BVH(Ray& ray, Scene& scene, bool isReflect, int max_depth,
	Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
	Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&), pcg32_state& rng)
{
	if (!FindNearstIntersection_BVH(ray, scene, m_BVH, m_Vars.t_min, m_Vars.t_max)) // Miss
	{
		return (this->*miss)(ray, scene, m_Vars, max_depth);
	}
	else
	{
		if (isReflect)
			return HitNearst_BVH(ray, scene, true, max_depth, miss, illumination, rng);
		else
			return HitNearst_BVH(ray, scene, false, max_depth, miss, illumination, rng);
	}
}


Vector3 Renderer::HitPointOriginal(const Ray& ray, Shape& NearstObj)
{

	Vector4 hit_point = Vector4(ray.Origin + ray.distance * ray.Direction, 1.);
	//Vector3 hit_point = NearstObj->InverseTransform * hit_point_original;
	Vector3 hit_point_original;
	if (NearstObj.index() == 0) // is sphere
	{
		ParsedSphere &sphere = std::get<ParsedSphere>(NearstObj);
		Matrix4x4 transform = inverse(translate(Vector3(sphere.position.x, sphere.position.y, sphere.position.z)));
		Vector4 hit_point_original_4 = transform * hit_point;
		hit_point_original = Vector3(hit_point_original_4.x, hit_point_original_4.y, hit_point_original_4.z);
	}
	else
	{
		hit_point_original = Vector3(hit_point.x, hit_point.y, hit_point.z);
	}
	return hit_point_original;
}
Vector3 Renderer::HitNearst(Ray& ray, Scene& scene, bool isReflect, int max_depth,
	Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
	Vector3 (Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&), pcg32_state& rng)
{
	Shape& NearstObj = *ray.object;
	Vector3 hit_point_original = HitPointOriginal(ray, NearstObj);
	Vector3 hit_point = ray.Origin + ray.distance * ray.Direction;

	int material_id;
	Vector3 normal_transformed;
	if (NearstObj.index() == 0) // is sphere
	{
		ParsedSphere& sphere = std::get<ParsedSphere>(NearstObj);
		Vector4 normal_original = GetNormalByHitPoint(hit_point_original, sphere);
		Vector4 normal_transformed_4 = Vector4(transpose(inverse(
			translate(Vector3(sphere.position.x, sphere.position.y, sphere.position.z)))) * normal_original);
		normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));

		material_id = sphere.material_id;

	}
	else //is triangle
	{
		Triangle& triangle = std::get<Triangle>(NearstObj);
		//if(triangle.mesh->normals.size() > triangle.index) // has cached normal
		//{
		//	normal_transformed = triangle.mesh->normals[triangle.index];
		//}
		//else
		//{
		Vector4 normal_transformed_4 = GetNormalByHitPoint(hit_point_original, triangle);
		normal_transformed = Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z);

		material_id = triangle.mesh->material_id;
	}

	//ray.Origin = hit_point + normal_transformed * 0.001;

	Vector3 color = (this->*illumination)(ray, isReflect, hit_point, normal_transformed, NearstObj, scene, m_Vars);

	ray.Origin = hit_point;
	ray.object.reset();
	ray.distance = 0.;
	if (isReflect)
	{
		ParsedMaterial material = scene.materials[material_id];
		ParsedColor parsed_color = std::visit([=](auto& m) {return m.reflectance; }, material);
		Vector3 surface_color = std::get<Vector3>(parsed_color);

		//Real noize_theta = next_pcg32_real<Real>(rng) * c_PI;
		//Real noize_phi = next_pcg32_real<Real>(rng) * 2 * c_PI;
		//Vector3 sample_in_unit_sphere{ std::sin(noize_theta) * std::cos(noize_phi),
		//std::cos(noize_theta),
		//std::sin(noize_theta) * std::sin(noize_phi) };
		

		if(material.index() == 0) // diffuse
		{
			//if(length_squared(sample_in_unit_sphere - Vector3(0., 0., -1.)) > 1e-4)
			//	ray.Direction = normal_transformed + sample_in_unit_sphere;
			//else
			//	ray.Direction = normal_transformed;
			return color;
		}
		else if (material.index() == 1) // mirror
		{
			ray.Direction = reflect(ray.Direction, normal_transformed);
			// Fuzzed
			//ray.Direction += 0.1*sample_in_unit_sphere;
			//ray.Direction = normalize(ray.Direction);
		}

		else if (material.index() == 2) // plastic
		{
			ParsedPlastic material_plastic = std::get<ParsedPlastic>(material);
			Real index_of_refraction = material_plastic.eta;

			if (!isFrontFace(ray, normal_transformed))
			{
				index_of_refraction = 1. / material_plastic.eta;
				normal_transformed = -normal_transformed;
			}

			double cos_theta = min(dot(-ray.Direction, normal_transformed), 1.0);
			double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

			if (index_of_refraction * sin_theta > 1.0) {	
				// Must Reflect
				ray.Direction = reflect(ray.Direction, normal_transformed);
			}
			else {
				Real kr = reflectance(cos_theta, index_of_refraction);
				Real kt = 1 - kr;

				Ray refracted_ray = { ray.Origin, refract(ray.Direction, normal_transformed, index_of_refraction) };
				// Can Refract
				ray.Direction = reflect(ray.Direction, normal_transformed);

				if (0 == max_depth--)
					return (this->*miss)(ray, scene, m_Vars, max_depth);
				else
				{
					return  color + surface_color * (kt * TraceRay(refracted_ray, scene, true, max_depth, miss, illumination, rng) 
						+ kr * TraceRay(ray, scene, true, max_depth, miss, illumination, rng));
				}

			}
			
		}
		if (0 == max_depth--)
			return color;
		else
			return  color + surface_color * TraceRay(ray, scene, true, max_depth, miss, illumination, rng);
	}
	//else
	//{
	//	if (*remain_refract == refraction_limits)
	//		return NearstObj->transition * TraceRay(ray, scene, false, remain_reflect, remain_refract, refraction_limits);
	//	else if (0 == (*remain_refract)--)
	//		return color;
	//	else
	//		return  color + NearstObj->transition * TraceRay(ray, scene, false, remain_reflect, remain_refract, refraction_limits);
	//}
}

Vector3 Renderer::HitNearst_BVH(Ray& ray, Scene& scene, bool isReflect, int max_depth,
	Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
	Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&), pcg32_state& rng)
{
	Shape& NearstObj = *ray.object;
	Vector3 hit_point_original = HitPointOriginal(ray, NearstObj);
	Vector3 hit_point = ray.Origin + ray.distance * ray.Direction;


	int material_id;
	Vector3 normal_transformed;
	if (NearstObj.index() == 0) // is sphere
	{
		ParsedSphere& sphere = std::get<ParsedSphere>(NearstObj);
		Vector4 normal_original = GetNormalByHitPoint(hit_point_original, sphere);
		Vector4 normal_transformed_4 = Vector4(transpose(inverse(
			translate(Vector3(sphere.position.x, sphere.position.y, sphere.position.z)))) * normal_original);
		normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));

		material_id = sphere.material_id;

	}
	else //is triangle
	{
		Triangle& triangle = std::get<Triangle>(NearstObj);
		//if(triangle.mesh->normals.size() > triangle.index) // has cached normal
		//{
		//	normal_transformed = triangle.mesh->normals[triangle.index];
		//}
		//else
		//{
		Vector4 normal_transformed_4 = GetNormalByHitPoint(hit_point_original, triangle);
		normal_transformed = Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z);

		material_id = triangle.mesh->material_id;
	}

	//ray.Origin = hit_point + normal_transformed * 0.001;

	Vector3 color = (this->*illumination)(ray, isReflect, hit_point, normal_transformed, NearstObj, scene, m_Vars);

	ray.Origin = hit_point;
	ray.object.reset();
	ray.distance = 0.;
	if (isReflect)
	{
		//if (NearstObj.index() == 0) // sphere
		//	material_id = std::get<ParsedSphere>(NearstObj).material_id;
		//else // triangle
		//	material_id = std::get<Triangle>(NearstObj).mesh->material_id;

		ParsedMaterial material = scene.materials[material_id];
		ParsedColor parsed_color = std::visit([=](auto& m) {return m.reflectance; }, material);
		Vector3 surface_color = std::get<Vector3>(parsed_color);

		//Real noize_theta = next_pcg32_real<Real>(rng) * c_PI;
		//Real noize_phi = next_pcg32_real<Real>(rng) * 2 * c_PI;
		//Vector3 sample_in_unit_sphere{ std::sin(noize_theta) * std::cos(noize_phi),
		//std::cos(noize_theta),
		//std::sin(noize_theta) * std::sin(noize_phi) };


		if (material.index() == 0) // diffuse
		{
			//if(length_squared(sample_in_unit_sphere - Vector3(0., 0., -1.)) > 1e-4)
			//	ray.Direction = normal_transformed + sample_in_unit_sphere;
			//else
			//	ray.Direction = normal_transformed;
			return color;
		}
		else if (material.index() == 1) // mirror
		{
			ray.Direction = reflect(ray.Direction, normal_transformed);
			// Fuzzed
			//ray.Direction += 0.1*sample_in_unit_sphere;
			//ray.Direction = normalize(ray.Direction);
		}

		else if (material.index() == 2) // plastic
		{
			ParsedPlastic material_plastic = std::get<ParsedPlastic>(material);
			Real index_of_refraction = material_plastic.eta;

			if (!isFrontFace(ray, normal_transformed))
			{
				index_of_refraction = 1. / material_plastic.eta;
				normal_transformed = -normal_transformed;
			}

			double cos_theta = min(dot(-ray.Direction, normal_transformed), 1.0);
			double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

			if (index_of_refraction * sin_theta > 1.0) {
				// Must Reflect
				ray.Direction = reflect(ray.Direction, normal_transformed);
			}
			else {
				Real kr = reflectance(cos_theta, index_of_refraction);
				Real kt = 1 - kr;

				Ray refracted_ray = { ray.Origin, refract(ray.Direction, normal_transformed, index_of_refraction) };
				// Can Refract
				ray.Direction = reflect(ray.Direction, normal_transformed);

				if (0 == max_depth--)
					return (this->*miss)(ray, scene, m_Vars, max_depth);
				else
				{
					return  color + surface_color * (kt * TraceRay_BVH(refracted_ray, scene, true, max_depth, miss, illumination, rng)
						+ kr * TraceRay_BVH(ray, scene, true, max_depth, miss, illumination, rng));
				}

			}

		}
		if (0 == max_depth--)
			return color;
		else
			return  color + surface_color * TraceRay_BVH(ray, scene, true, max_depth, miss, illumination, rng);
	}
	//else
	//{
	//	if (*remain_refract == refraction_limits)
	//		return NearstObj->transition * TraceRay(ray, scene, false, remain_reflect, remain_refract, refraction_limits);
	//	else if (0 == (*remain_refract)--)
	//		return color;
	//	else
	//		return  color + NearstObj->transition * TraceRay(ray, scene, false, remain_reflect, remain_refract, refraction_limits);
	//}
}


//Vector3 Renderer::HitNearst_AABB(Ray& ray, Scene& scene, bool isReflect, int max_depth,
//	Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
//	Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&), pcg32_state& rng)
//{
//	Shape& NearstObj = scene.shapes[ray.Objects];
//	Vector3 hit_point_original = HitPointOriginal(ray, NearstObj);
//	Vector3 hit_point = ray.Origin + ray.Distances * ray.Direction;
//
//	AxisAlignedBoundingBox aabb;
//	Vector3 normal_transformed;
//	if (NearstObj.index() == 0) // is sphere
//	{
//		ParsedSphere& sphere = std::get<ParsedSphere>(NearstObj);
//		aabb = GetAabbByShape(sphere);
//	}
//	else //is triangle
//	{
//		ParsedTriangleMesh& triangle = std::get<ParsedTriangleMesh>(NearstObj);
//		aabb = GetAabbByShape(triangle, ray.NearstIndexinMesh);
//	}
//
//	Vector3 color = illumination(ray, isReflect, hit_point, normal_transformed, NearstObj, scene, m_Vars);
//}
Vector3 Renderer::Miss(const Ray& ray, Scene& scene, int max_depth)
{
	return Vector3(0., 0., 0.);
	//return Vector4(0.4f, 0.9f, 1.0f, 0.9f);
	//return Vector4(0.2f, 0.45f, 0.5f, 0.9f);
}


bool Renderer::FindNearstIntersection(Ray& ray, Scene& scene, Real t_min, Real t_max)
{
	for (Shape& shape : scene.shapes)
	{
		if (ray.FindIntersection(shape, t_min, t_max))
		{
			t_min = t_min;
			t_max = ray.distance;
		}
	}
	if(ray.object) // hit
		return true;
	else
		return false;
}

bool Renderer::FindNearstIntersection_AABB(Ray& ray, Scene& scene, std::vector<AxisAlignedBoundingBox>& boxs)
{
	Real t_nearst = 15000.;
	Real epsilon = 1e-4;
	Real distance = 0;
	for (AxisAlignedBoundingBox& box : boxs)
	{
		if (box.isHit(ray, epsilon, 15000.))
		{
			//ray.Objects[0] = std::make_shared<Shape>(scene.shapes[0]);
			return true;
		}
	}
	return false;
}


bool Renderer::FindNearstIntersection_BVH(Ray& ray, Scene& scene, BVH& node, Real t_min, Real t_max)
{
	if (node.isHit(ray, t_min, t_max))
	{
		if (ray.object)
			return true;
	}
	
	return false;
}

//Vector3 Renderer::Illumination(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene)
//{
//	Vector4 color = Vector4(0.);
//	Vector3 light_direction = Vector3(0.);
//
//	Vector4 ambient = NearstObj->ambient;
//	Vector4 emission = NearstObj->emission;
//
//	for (Light* light : scene.Lights)
//	{
//		float d = 0;
//		float visibility = 1.;
//
//		if (light->type == DIRECTIONALLIGHT)
//		{
//			DirectionalLight* direcLight = DirectionalLight::LightToDirLight(light);
//			light_direction = direcLight->light_direction;
//			d = 15000.;
//		}
//		else
//		{
//			PointLight* pointLight = PointLight::LightToPointLight(light);
//			light_direction = ray.Origin - pointLight->light_position;
//			d = sqrt(dot(light_direction, light_direction));
//		}
//
//		Ray shadow_ray = { ray.Origin, -normalize(light_direction) };
//		FindNearstIntersection(shadow_ray, scene);
//
//
//		if (shadow_ray.Objects == -1)
//			visibility = 1.;
//		else if (shadow_ray.Objects != -1 && shadow_ray.Distances > d)
//			visibility = 1.;
//		else
//			visibility = 0;
//
//		light_direction = normalize(light_direction);
//
//		//For rainbow color
//		//light->light_color = vec4(sin((ray.Origin.x + 10.) / (12.7 / 1.5)), sin((ray.Origin.z + 50.) / (31.8 / 1.5)), cos((ray.Origin.x + 10.) / (12.7 / 1.5)), 1.);
//
//		Vector4 diffusion = light->light_color * NearstObj->diffuse *
//			glm::max(glm::dot(-light_direction, normalize(Normal)), 0.);
//
//		Vector3 direction = normalize(ray.Direction);
//		Vector3 halfvec = normalize(-direction - light_direction);
//		Vector4 specular = light->light_color * NearstObj->specular *
//			pow(glm::max(glm::dot(halfvec, normalize(Normal)), 0.), NearstObj->shininess);
//
//		if (light->type == POINTLIGHT)
//		{
//			specular = specular / (attenuation[0] + attenuation[1] * d + attenuation[2] * d * d);
//			diffusion = diffusion / (attenuation[0] + attenuation[1] * d + attenuation[2] * d * d);
//		}
//
//		color += (diffusion + specular) * visibility;
//	}
//
//	color += ambient + emission;
//
//	return color;
//}
bool isFrontFace(const Ray& ray, const Vector3& normal)
{
	return (dot(ray.Direction, normal) < 0);
}

Real reflectance(Real cosine, Real ref_idx)
{
	auto r0 = (1 - ref_idx) / (1 + ref_idx);
	r0 = r0 * r0;
	return r0 + (1 - r0) * pow((1 - cosine), 5);
}
Vector3 Renderer::Miss_hw_1_1(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{

	return Vector3(ray.Direction.x, ray.Direction.y, 0.);
}
Vector3 Renderer::Illumination_hw_1_1(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	Vector3 color(1., 0., 0.);
	return color;
}

Vector3 Renderer::Miss_hw_1_2(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return Vector3(0.5, 0.5, 0.5);
}
Vector3 Renderer::Illumination_hw_1_2(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	return (Normal + Vector3(1., 1., 1.)) / 2.;
}

Vector3 Renderer::Miss_hw_1_3(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return Vector3(0.5, 0.5, 0.5);
}
Vector3 Renderer::Illumination_hw_1_3(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	return (Normal + Vector3(1., 1., 1.)) / 2.;
}

Vector3 Renderer::Miss_hw_1_4(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return Vector3(0.5, 0.5, 0.5);
}
Vector3 Renderer::Illumination_hw_1_4(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	ParsedMaterial material = scene.materials[material_id];
	ParsedColor parsed_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 color = std::get<Vector3>(parsed_color);
	return color;
}

Vector3 Renderer::Miss_hw_1_5(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return Vector3(0.5, 0.5, 0.5);
}
Vector3 Renderer::Illumination_hw_1_5(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	ParsedMaterial material = scene.materials[material_id];
	ParsedColor parsed_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color = std::get<Vector3>(parsed_color);

	Vector3 light_direction = Vector3(0., 0., 0.);

	Vector3 color{0., 0., 0.};
	for (ParsedLight light : scene.lights)
	{
		Real d = 0;
		Real visibility = 1.;
		Vector3 position;
		Vector3 intensity;

		if (light.index() == 0) // is PointLight
		{
			ParsedPointLight point_light = std::get<ParsedPointLight>(light);
			position = point_light.position;
			intensity = point_light.intensity;
			light_direction = HitPoint - point_light.position;
			d = length(light_direction);
		}
		else{} // is AreaLight

		Ray shadow_ray = { HitPoint, -normalize(light_direction) };

		if (!Renderer::FindNearstIntersection(shadow_ray, scene, 1e-4, d))
			visibility = 1.;
		else
			visibility = 0.;

		light_direction = normalize(light_direction);

		//For rainbow color
		//light->light_color = vec4(sin((ray.Origin.x + 10.) / (12.7 / 1.5)), sin((ray.Origin.z + 50.) / (31.8 / 1.5)), cos((ray.Origin.x + 10.) / (12.7 / 1.5)), 1.);

		Vector3 diffuse;
		Vector3 normal = normalize(Normal);
		if(!isFrontFace(ray, normal))
		{
			normal = -normal;
		}
		diffuse = intensity * surface_color *
			max(dot(-light_direction, normal), 0.);

		Vector3 direction = normalize(ray.Direction);

		diffuse = diffuse / (vars.attenuation_const + vars.attenuation_linear * d + vars.attenuation_quadratic * d * d);
		color += diffuse  * visibility * c_INVPI;
	}

	//color += ambient + emission;

	return color;
}

Vector3 Renderer::Miss_hw_1_6(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return Miss_hw_1_5(ray, scene, vars, max_depth);
}
Vector3 Renderer::Illumination_hw_1_6(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	return Illumination_hw_1_5(vis_ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars);
}

Vector3 Renderer::Miss_hw_1_7(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	//if (*remain_reflect > 0)
	//	return Vector3(0., 0., 0.);
	return Vector3(0.5, 0.5, 0.5);
}

Vector3 Renderer::Illumination_hw_1_7(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{

	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	ParsedMaterial material = scene.materials[material_id];
	ParsedColor parsed_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color = std::get<Vector3>(parsed_color);

	Vector3 light_direction = Vector3(0., 0., 0.);

	Vector3 color{ 0., 0., 0. };
	for (ParsedLight light : scene.lights)
	{
		Real d = 0;
		Real visibility = 1.;
		Vector3 position;
		Vector3 intensity;

		if (light.index() == 0) // is PointLight
		{
			ParsedPointLight point_light = std::get<ParsedPointLight>(light);
			position = point_light.position;
			intensity = point_light.intensity;
			light_direction = HitPoint - point_light.position;
			d = length(light_direction);
		}
		else {} // is AreaLight

		Ray shadow_ray = { HitPoint, -normalize(light_direction) };
		if (!Renderer::FindNearstIntersection(shadow_ray, scene, 1e-4, d))
			visibility = 1.;
		else
			visibility = 0;

		light_direction = normalize(light_direction);

		//For rainbow color
		//light->light_color = vec4(sin((ray.Origin.x + 10.) / (12.7 / 1.5)), sin((ray.Origin.z + 50.) / (31.8 / 1.5)), cos((ray.Origin.x + 10.) / (12.7 / 1.5)), 1.);

		Vector3 diffuse;
		Vector3 normal = normalize(Normal);
		if (dot(ray.Direction, normal) > 0)
		{
			normal = -normal;
		}
		diffuse = intensity * surface_color *
			max(dot(-light_direction, normal), 0.);

		Vector3 direction = normalize(ray.Direction);

		Vector3 halfvec = normalize(-direction - light_direction);
		Vector3 specular = intensity * surface_color * 
			pow(max(dot(halfvec, normalize(normal)), 0.), 1000000);

		diffuse = diffuse / (vars.attenuation_const + vars.attenuation_linear * d + vars.attenuation_quadratic * d * d);
		specular = specular / (vars.attenuation_const + vars.attenuation_linear * d + vars.attenuation_quadratic * d * d);
		if (material.index() == 0)// diffuse
			color += diffuse * visibility * c_INVPI;
		else if (material.index() == 1) // mirror
			//color += specular * visibility * c_INVPI;
			;
	}

	//color += ambient + emission;

	return color;
}

Vector3 Renderer::Miss_hw_1_8(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return Miss_hw_1_7(ray, scene, vars, max_depth);
}
Vector3 Renderer::Illumination_hw_1_8(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	return Illumination_hw_1_7(ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars);
}




Vector3 Renderer::Miss_hw_1_10(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	
	auto t = (-ray.Direction.y + 2.0) * 0.73;
	auto v = (ray.Direction.z ) / 2;
	return (t*t + v) * Vector3(0.6, 0.9, 0.95);
	//return Vector3(0.5, 0.5, 0.5);
}

Vector3 Renderer::Illumination_hw_1_10(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{

	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	ParsedMaterial material = scene.materials[material_id];
	ParsedColor parsed_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color = std::get<Vector3>(parsed_color);

	Vector3 light_direction = Vector3(0., 0., 0.);

	Vector3 color{ 0., 0., 0. };
	//parallel_for([&](int counts)
	//	{

	//	}, (int64_t)10);
	for (ParsedLight light : scene.lights)
	{
		Real d = 0;
		Real visibility = 1.;
		Vector3 position;
		Vector3 intensity;

		if (light.index() == 0) // is PointLight
		{
			ParsedPointLight point_light = std::get<ParsedPointLight>(light);
			position = point_light.position;
			intensity = point_light.intensity;
			light_direction = HitPoint - point_light.position;
			d = length(light_direction);
		}
		else {} // is AreaLight

		Ray shadow_ray = { HitPoint, -normalize(light_direction) };
		if (!Renderer::FindNearstIntersection(shadow_ray, scene, 1e-4, d))
			visibility = 1.;
		else
			visibility = 0;

		light_direction = normalize(light_direction);

		//For rainbow color
		//light->light_color = vec4(sin((ray.Origin.x + 10.) / (12.7 / 1.5)), sin((ray.Origin.z + 50.) / (31.8 / 1.5)), cos((ray.Origin.x + 10.) / (12.7 / 1.5)), 1.);

		Vector3 diffuse;
		Vector3 normal = normalize(Normal);
		if (dot(ray.Direction, normal) > 0)
		{
			normal = -normal;
		}
		diffuse = intensity * surface_color *
			max(dot(-light_direction, normal), 0.);

		Vector3 direction = normalize(ray.Direction);

		Vector3 halfvec = normalize(-direction - light_direction);
		Vector3 specular = intensity * surface_color *
			pow(max(dot(halfvec, normalize(normal)), 0.), 1000000);

		Vector3 transmission{ 0., 0., 0. };

		diffuse = diffuse / (vars.attenuation_const + vars.attenuation_linear * d + vars.attenuation_quadratic * d * d);
		specular = specular / (vars.attenuation_const + vars.attenuation_linear * d + vars.attenuation_quadratic * d * d);
		if (material.index() == 0)// diffuse
			color += diffuse * visibility * c_INVPI;
		else if (material.index() == 1) // mirror
			//color += specular * visibility * c_INVPI;
			;
		else if (material.index() == 2) // plastic
			color += transmission * visibility * c_INVPI;
	}

	//color += ambient + emission;

	return color;
}

Vector3 Renderer::Miss_hw_1_11(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{

	//auto t = (ray.Direction.y + 1.0) / 3;
	//auto v = (ray.Direction.z ) / 3;
	//return (1.0 - t - v) * Vector3(1.0, 1.0, 1.0) + (t + v) * Vector3(0.5, 0.7, 1.0);
	return Miss_hw_1_10(ray, scene, vars, max_depth);
}

Vector3 Renderer::Illumination_hw_1_11(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	return Illumination_hw_1_10(ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars);
}

Vector3 Renderer::Miss_hw_1_9(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{

	auto t = (-ray.Direction.y + 2.0) * 0.73;
	auto v = (ray.Direction.z) / 2;
	return (t * t + v) * Vector3(0.6, 0.9, 0.95);
	//return Vector3{ 0.2, 0.2, 0.2};
	//return Miss_hw_1_10(ray, scene, vars, max_depth);
}

Vector3 Renderer::Illumination_hw_1_9(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	return Illumination_hw_1_10(ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars);
}


Vector3 Renderer::Miss_hw_2_1(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{

	//auto t = (-ray.Direction.y + 2.0) * 0.73;
	//auto v = (ray.Direction.z) / 2;
	//return (t * t + v) * Vector3(0.6, 0.9, 0.95);
	return Vector3(0.5, 0.5, 0.5);
}

Vector3 Renderer::Illumination_hw_2_1(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	Triangle& triangle = std::get<Triangle>(NearstObj);
	Vector3 p1 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][0]];
	Vector3 p2 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][1]];
	Vector3 p3 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][2]];
	Vector3 s[2];
	for (int i = 2; i--; ) 
	{ s[i][0] = p3[i] - p1[i];
		s[i][1] = p2[i] - p1[i];
		s[i][2] = p1[i] - HitPoint[i];
	}
	Vector3 u = cross(s[0], s[1]);
	return Vector3(1. - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
}

Vector3 Renderer::Miss_hw_2_2(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return Miss_hw_2_1(ray, scene, vars, max_depth);
}

Vector3 Renderer::Illumination_hw_2_2(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	return Illumination_hw_2_1(ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars);
}

Vector3 Renderer::Miss_hw_2_3(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_2_3(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	return Illumination_hw_1_10(ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars);
}

Vector3 Renderer::Miss_hw_2_4(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_2_4(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	return Vector3(1., 1., 1.);
}

Vector3 Renderer::Miss_hw_2_5(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_2_5(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	ParsedMaterial material = scene.materials[material_id];
	ParsedColor parsed_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color = std::get<Vector3>(parsed_color);

	Vector3 light_direction = Vector3(0., 0., 0.);

	Vector3 color{ 0., 0., 0. };
	//parallel_for([&](int counts)
	//	{

	//	}, (int64_t)10);
	for (ParsedLight light : scene.lights)
	{
		Real d = 0;
		Real visibility = 1.;
		Vector3 position;
		Vector3 intensity;

		if (light.index() == 0) // is PointLight
		{
			ParsedPointLight point_light = std::get<ParsedPointLight>(light);
			position = point_light.position;
			intensity = point_light.intensity;
			light_direction = HitPoint - point_light.position;
			d = length(light_direction);
		}
		else {} // is AreaLight

		Ray shadow_ray = { HitPoint, -normalize(light_direction) };
		if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, d))
			visibility = 1.;
		else
			visibility = 0;

		light_direction = normalize(light_direction);

		//For rainbow color
		//light->light_color = vec4(sin((ray.Origin.x + 10.) / (12.7 / 1.5)), sin((ray.Origin.z + 50.) / (31.8 / 1.5)), cos((ray.Origin.x + 10.) / (12.7 / 1.5)), 1.);

		Vector3 diffuse;
		Vector3 normal = normalize(Normal);
		if (dot(ray.Direction, normal) > 0)
		{
			normal = -normal;
		}
		diffuse = intensity * surface_color *
			max(dot(-light_direction, normal), 0.);

		Vector3 direction = normalize(ray.Direction);

		Vector3 halfvec = normalize(-direction - light_direction);
		Vector3 specular = intensity * surface_color *
			pow(max(dot(halfvec, normalize(normal)), 0.), 1000000);

		Vector3 transmission{ 0., 0., 0. };

		diffuse = diffuse / (vars.attenuation_const + vars.attenuation_linear * d + vars.attenuation_quadratic * d * d);
		specular = specular / (vars.attenuation_const + vars.attenuation_linear * d + vars.attenuation_quadratic * d * d);
		if (material.index() == 0)// diffuse
			color += diffuse * visibility * c_INVPI;
		else if (material.index() == 1) // mirror
			//color += specular * visibility * c_INVPI;
			;
		else if (material.index() == 2) // plastic
			color += transmission * visibility * c_INVPI;
	}

	//color += ambient + emission;

	return color;
}