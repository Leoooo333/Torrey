//#pragma comment(lib, "FreeImage.lib")
#include "Renderer.h"

#include "parallel.h"
#include "3rdparty\pcg.h"
#include "Shape.h"
#include "timer.h"
#include "transform.h"

#define TILE_SIZE 64
void Renderer::Render(CameraUnion& camera, Variables& vars, ParsedScene& scene,
                      Vector3(*miss)(const Ray&, ParsedScene&, Variables&, int*, int*),
                      Vector3(*illumination)(Ray&, bool, Vector3, Vector3, ParsedShape&, ParsedScene&, Variables&))
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

					Ray ray_reflect = std::visit([=](auto& cam) {return cam.CaculateRayDirections(x, y); }, camera);
					Ray ray_refract = std::visit([=](auto& cam) {return cam.CaculateRayDirections(x, y); }, camera);
					int* remain_reflect = new int(m_Vars.maxdepth);
					int* remain_refract = new int(m_Vars.maxdepth_refract);

					//Reflect only
					color = TraceRay(ray_reflect, scene, true, remain_reflect, remain_refract, m_Vars.maxdepth_refract, miss, illumination, rng);

					//Reflect && refract
			/*		color = TraceRay(ray_reflect, m_Vars.scene_global, true, remain_reflect, remain_refract, m_Vars.maxdepth_refract) +
						TraceRay(ray_refract, m_Vars.scene_global, false, remain_reflect, remain_refract, m_Vars.maxdepth_refract);*/

					(*m_Image)(x, height - 1 - y) = color;

					//m_Vars.isPrimary_ray = true;
				}
				if(y % 50 == 0)
				{
					std::cout << "Finish:	" << (float)y / (float)height << std::endl;
				}
			}
			}, Vector2i(num_tiles_x, num_tiles_y));
		m_RenderTime = tick(start_time);
		std::cout << "Total Time:	" << m_RenderTime << std::endl;
	}
	else if(samples_per_pixel > 1)
	{
		const int tile_size = vars.tile_size;
		int num_tiles_x = (width + tile_size - 1) / tile_size;
		int num_tiles_y = (height + tile_size - 1) / tile_size;

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
						Ray ray_reflect = std::visit([=](auto& cam) {return cam.CaculateRayDirections(x, y, offset); }, camera);
						Ray ray_refract = std::visit([=](auto& cam) {return cam.CaculateRayDirections(x, y, offset); }, camera);
						int* remain_reflect = new int(m_Vars.maxdepth);
						int* remain_refract = new int(m_Vars.maxdepth_refract);

						//Reflect only
						color += TraceRay(ray_reflect, scene, true, remain_reflect, remain_refract, m_Vars.maxdepth_refract, miss, illumination, rng);

						//Reflect && refract
				/*		color = TraceRay(ray_reflect, m_Vars.scene_global, true, remain_reflect, remain_refract, m_Vars.maxdepth_refract) +
							TraceRay(ray_refract, m_Vars.scene_global, false, remain_reflect, remain_refract, m_Vars.maxdepth_refract);*/

					}
					color /= (Real)samples_per_pixel;
					(*m_Image)(x, height - 1 - y) = color;

					//m_Vars.isPrimary_ray = true;
				}
				if (y % 50 == 0)
				{
					std::cout << "Finish:	" << (float)y / (float)height << std::endl;
				}
			}
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

Vector3 Renderer::TraceRay(Ray& ray, ParsedScene& scene, bool isReflect, int* remain_reflect, int* remain_refract, int refraction_limits, 
	Vector3 (*miss)(const Ray&, ParsedScene&, Variables&, int*, int*),
	Vector3(*illumination)(Ray&, bool, Vector3, Vector3, ParsedShape&, ParsedScene&, Variables&), pcg32_state& rng)
{
	FindNearstIntersection(ray, scene);
	if (ray.t_nearst == 15000.) // Miss
	{
		return miss(ray, scene, m_Vars, remain_reflect, remain_refract);
	}
	else
	{
		if (isReflect)
			return HitNearst(ray, scene, true, remain_reflect, remain_refract, refraction_limits, miss, illumination, rng);
		else
			return HitNearst(ray, scene, false, remain_reflect, remain_refract, refraction_limits, miss, illumination, rng);
	}
}

Vector3 Renderer::HitPointOriginal(const Ray& ray, ParsedShape& NearstObj)
{

	Vector4 hit_point = Vector4(ray.Origin + ray.t_nearst * ray.Direction, 1.);
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
Vector3 Renderer::HitNearst(Ray& ray, ParsedScene& scene, bool isReflect, int* remain_reflect, int* remain_refract, int refraction_limits, 
	Vector3(*miss)(const Ray&, ParsedScene&, Variables&, int*, int*),
	Vector3 (*illumination)(Ray&, bool, Vector3, Vector3, ParsedShape&, ParsedScene&, Variables&), pcg32_state& rng)
{
	ParsedShape& NearstObj = scene.shapes[ray.NearstObject];
	Vector3 hit_point_original = HitPointOriginal(ray, NearstObj);
	Vector3 hit_point = ray.Origin + ray.t_nearst * ray.Direction;


	Vector3 normal_transformed;
	if (NearstObj.index() == 0) // is sphere
	{
		ParsedSphere& sphere = std::get<ParsedSphere>(NearstObj);
		Vector4 normal_original = GetNormalByHitPoint(hit_point_original, sphere);
		Vector4 normal_transformed_4 = Vector4(transpose(inverse(
			translate(Vector3(sphere.position.x, sphere.position.y, sphere.position.z)))) * normal_original);
		normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
	}
	//else //is triangle
	//{
	//	ParsedTriangleMesh& triangle = std::get<ParsedTriangleMesh>(NearstObj);
	//	normal_transformed = triangle.GetNormalByHitPoint(hit_point_original, triangle);
	//}

	//ray.Origin = hit_point + normal_transformed * 0.001;

	Vector3 color = illumination(ray, isReflect, hit_point, normal_transformed, NearstObj, scene, m_Vars);

	ray.Origin = hit_point;
	if (isReflect)
	{
		int material_id = std::visit([=](auto& obj) {return obj.material_id; }, NearstObj);
		ParsedMaterial material = scene.materials[material_id];
		ParsedColor parsed_color = std::visit([=](auto& m) {return m.reflectance; }, material);
		Vector3 surface_color = std::get<Vector3>(parsed_color);

		Real noize_theta = next_pcg32_real<Real>(rng) * c_PI;
		Real noize_phi = next_pcg32_real<Real>(rng) * 2 * c_PI;
		Vector3 sample_in_unit_sphere{ std::sin(noize_theta) * std::cos(noize_phi),
		std::cos(noize_theta),
		std::sin(noize_theta) * std::sin(noize_phi) };
		

		if(material.index() == 0) // diffuse
		{
			//if(length_squared(sample_in_unit_sphere - Vector3(0., 0., -1.)) > 1e-4)
			//	ray.Direction = normal_transformed + sample_in_unit_sphere;
			//else
			//	ray.Direction = normal_transformed;
			return color;
		}
		else if (material.index() == 1) // mirror
			ray.Direction = reflect(ray.Direction, normal_transformed);

		if (0 == (*remain_reflect)--)
			return color;
		else
			return  color + surface_color * TraceRay(ray, scene, true, remain_reflect, remain_refract, refraction_limits, miss, illumination, rng);
	}
	//else
	//{
	//	ray.Direction = glm::refract(ray.Direction, normal_transformed, 1.05f);

	//	if (*remain_refract == refraction_limits)
	//		return NearstObj->transition * TraceRay(ray, scene, false, remain_reflect, remain_refract, refraction_limits);
	//	else if (0 == (*remain_refract)--)
	//		return color;
	//	else
	//		return  color + NearstObj->transition * TraceRay(ray, scene, false, remain_reflect, remain_refract, refraction_limits);
	//}
}

Vector3 Renderer::Miss(const Ray& ray, ParsedScene& scene, int* remain_reflect, int* remain_refract)
{
	return Vector3(0., 0., 0.);
	//return Vector4(0.4f, 0.9f, 1.0f, 0.9f);
	//return Vector4(0.2f, 0.45f, 0.5f, 0.9f);
}


void Renderer::FindNearstIntersection(Ray& ray, ParsedScene& scene)
{
	Real t_nearst = 15000.;
	Real epsilon = 1e-4;
	int index_nearst = -1;
	int index_current = 0;

	Real distance = 0;

	for (ParsedShape& object : scene.shapes)
	{
		if (object.index() == 0) // is sphere
		{
			ParsedSphere& sphere = std::get<ParsedSphere>(object);
			distance = FindIntersection(ray, sphere);
		}
		//else // is triangle
		//{
		//	ParsedTriangleMesh& triangle = std::get<ParsedTriangleMesh>(object);
		//	distance = FindIntersection(ray, triangle);
		//}

		if (t_nearst > distance && distance >= 0. &&(distance > epsilon && distance < (1-epsilon) * 15000.))
		{
			t_nearst = distance;
			index_nearst = index_current;
		}

		index_current += 1;
	}

	ray.NearstObject = index_nearst;
	ray.t_nearst = t_nearst;
}

//Vector3 Renderer::Illumination(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene)
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
//		if (shadow_ray.NearstObject == -1)
//			visibility = 1.;
//		else if (shadow_ray.NearstObject != -1 && shadow_ray.t_nearst > d)
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

Vector3 Miss_hw_1_1(const Ray& ray, ParsedScene& scene, Variables& vars, int* remain_reflect, int* remain_refract)
{

	return Vector3(ray.Direction.x, ray.Direction.y, 0.);
}
Vector3 Illumination_hw_1_1(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars)
{
	Vector3 color(1., 0., 0.);
	return color;
}

Vector3 Miss_hw_1_2(const Ray& ray, ParsedScene& scene, Variables& vars, int* remain_reflect, int* remain_refract)
{
	return Vector3(0.5, 0.5, 0.5);
}
Vector3 Illumination_hw_1_2(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars)
{
	return (Normal + Vector3(1., 1., 1.)) / 2.;
}

Vector3 Miss_hw_1_3(const Ray& ray, ParsedScene& scene, Variables& vars, int* remain_reflect, int* remain_refract)
{
	return Vector3(0.5, 0.5, 0.5);
}
Vector3 Illumination_hw_1_3(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars)
{
	return (Normal + Vector3(1., 1., 1.)) / 2.;
}

Vector3 Miss_hw_1_4(const Ray& ray, ParsedScene& scene, Variables& vars, int* remain_reflect, int* remain_refract)
{
	return Vector3(0.5, 0.5, 0.5);
}
Vector3 Illumination_hw_1_4(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars)
{
	int material_id = std::visit([=](auto& obj) {return obj.material_id; }, NearstObj);
	ParsedMaterial material = scene.materials[material_id];
	ParsedColor parsed_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 color = std::get<Vector3>(parsed_color);
	return color;
}

Vector3 Miss_hw_1_5(const Ray& ray, ParsedScene& scene, Variables& vars, int* remain_reflect, int* remain_refract)
{
	return Vector3(0.5, 0.5, 0.5);
}
Vector3 Illumination_hw_1_5(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars)
{
	
	int material_id = std::visit([=](auto& obj) {return obj.material_id; }, NearstObj);
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
		Renderer::FindNearstIntersection(shadow_ray, scene);


		if (shadow_ray.NearstObject == -1)
			visibility = 1.;
		else if (shadow_ray.NearstObject != -1 && shadow_ray.t_nearst > d)
			visibility = 1.;
		else
			visibility = 0;

		light_direction = normalize(light_direction);

		//For rainbow color
		//light->light_color = vec4(sin((ray.Origin.x + 10.) / (12.7 / 1.5)), sin((ray.Origin.z + 50.) / (31.8 / 1.5)), cos((ray.Origin.x + 10.) / (12.7 / 1.5)), 1.);

		Vector3 diffuse;
		Vector3 normal = normalize(Normal);
		if(dot(ray.Direction, normal) > 0)
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

Vector3 Miss_hw_1_6(const Ray& ray, ParsedScene& scene, Variables& vars, int* remain_reflect, int* remain_refract)
{
	return Miss_hw_1_5(ray, scene, vars, remain_reflect, remain_refract);
}
Vector3 Illumination_hw_1_6(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars)
{
	return Illumination_hw_1_5(vis_ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars);
}

Vector3 Miss_hw_1_7(const Ray& ray, ParsedScene& scene, Variables& vars, int* remain_reflect, int* remain_refract)
{
	//if (*remain_reflect > 0)
	//	return Vector3(0., 0., 0.);
	return Vector3(0.5, 0.5, 0.5);
}

Vector3 Illumination_hw_1_7(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars)
{

	int material_id = std::visit([=](auto& obj) {return obj.material_id; }, NearstObj);
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
		Renderer::FindNearstIntersection(shadow_ray, scene);


		if (shadow_ray.NearstObject == -1)
			visibility = 1.;
		else if (shadow_ray.NearstObject != -1 && shadow_ray.t_nearst > d)
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
		if(material.index() == 0)// diffuse
			color += diffuse * visibility * c_INVPI;
		else if(material.index() == 1) // mirror
			color += specular * visibility * c_INVPI;
	}

	//color += ambient + emission;

	return color;
}

Vector3 Miss_hw_1_8(const Ray& ray, ParsedScene& scene, Variables& vars, int* remain_reflect, int* remain_refract)
{
	return Miss_hw_1_7(ray, scene, vars, remain_reflect, remain_refract);
}
Vector3 Illumination_hw_1_8(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars)
{
	return Illumination_hw_1_7(ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars);
}