//#pragma comment(lib, "FreeImage.lib")
#include "Renderer.h"

#include "Shape.h"
#include "timer.h"
#include "transform.h"

void Renderer::Render(CameraUnion& camera, Variables& vars, ParsedScene& scene,
                      Vector3(*miss)(const Ray&, ParsedScene&),
                      Vector3(*illumination)(Ray&, bool, Vector3, Vector3, ParsedShape&, ParsedScene&))
{
	attenuation[0] = vars.attenuation_const;
	attenuation[1] = vars.attenuation_linear;
	attenuation[2] = vars.attenuation_quadratic;

	int width = std::visit([=](auto& cam) {return cam.m_CameraParameters.width; }, camera);
	int height = std::visit([=](auto& cam) {return cam.m_CameraParameters.height; }, camera);
	m_Image = std::make_shared<Image3>(width, height);
	std::vector<Ray>& rays = std::visit([&](auto& cam) {return cam.m_Rays; }, camera);

	Timer start_time;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			Vector3 color = Vector3(0., 0., 0.);
			Ray ray_reflect = rays[x + y * width];
			Ray ray_refract = rays[x + y * width];
			int* remain_reflect = new int(vars.maxdepth);
			int* remain_refract = new int(vars.maxdepth_refract);

			//Reflect only
			color = TraceRay(ray_reflect, scene, true, remain_reflect, remain_refract, vars.maxdepth_refract, miss, illumination);

			//Reflect && refract
	/*		color = TraceRay(ray_reflect, vars.scene_global, true, remain_reflect, remain_refract, vars.maxdepth_refract) +
				TraceRay(ray_refract, vars.scene_global, false, remain_reflect, remain_refract, vars.maxdepth_refract);*/

			(*m_Image)(x, height - 1 - y) = color;

			//vars.isPrimary_ray = true;
		}
		std::cout << "Finish:	" << (float)y  / (float)height << std::endl;
	}
	m_RenderTime = tick(start_time);
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
	Vector3 (*miss)(const Ray& ray, ParsedScene& scene),
	Vector3(*illumination)(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene))
{
	FindNearstIntersection(ray, scene);
	if (ray.t_nearst == 15000.)
	{
		return miss(ray, scene);
	}
	else
	{
		if (isReflect)
			return HitNearst(ray, scene, true, remain_reflect, remain_refract, refraction_limits, miss, illumination);
		else
			return HitNearst(ray, scene, false, remain_reflect, remain_refract, refraction_limits, miss, illumination);
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
	Vector3(*miss)(const Ray& ray, ParsedScene& scene),
	Vector3 (*illumination)(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene))
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

	ray.Origin = hit_point + normal_transformed * 0.001;
	Vector3 color = illumination(ray, isReflect, hit_point, normal_transformed, NearstObj, scene);
	if (isReflect)
	{
		ray.Direction = reflect(ray.Direction, normal_transformed);
		if (0 == (*remain_reflect)--)
			return color;
		else
			return  color + TraceRay(ray, scene, true, remain_reflect, remain_refract, refraction_limits, miss, illumination);
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

Vector3 Renderer::Miss(const Ray& ray, ParsedScene& scene)
{
	return Vector3(0., 0., 0.);
	//return Vector4(0.4f, 0.9f, 1.0f, 0.9f);
	//return Vector4(0.2f, 0.45f, 0.5f, 0.9f);
}



void Renderer::FindNearstIntersection(Ray& ray, ParsedScene& scene)
{
	float t_nearst = 15000.;
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

		if (t_nearst > distance && distance >= 0.)
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

Vector3 Miss_hw_1_1(const Ray& ray, ParsedScene& scene)
{

	return Vector3(ray.Direction.x, ray.Direction.y, 0.);
}
Vector3 Illumination_hw_1_1(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene)
{
	Vector3 color(1., 0., 0.);
	return color;
}

Vector3 Miss_hw_1_2(const Ray& ray, ParsedScene& scene)
{
	return Vector3(0.5, 0.5, 0.5);
}
Vector3 Illumination_hw_1_2(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene)
{
	return (Normal + Vector3(1., 1., 1.)) / 2.;
}