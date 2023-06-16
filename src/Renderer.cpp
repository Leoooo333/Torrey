//#pragma comment(lib, "FreeImage.lib")
#include "Renderer.h"

#include "parallel.h"
#include "3rdparty\pcg.h"
#include "Shape.h"
#include "timer.h"
#include "transform.h"
#include "progressreporter.h"
#include "3rdparty/stb_image.h"
#include "parse_scene.cpp"
#include <compute_normals.cpp>
#include <numeric>


#define TILE_SIZE 64


class BRDF_Sampler : public Sampler
{
public:
	Vector3 GetBRDF(Material material, Ray ray, Vector3 normal, Variables& m_Vars, const Vector3 direction, Scene& scene, BVH& node,  pcg32_state& rng, bool is_reflect, bool is_refract)
	{
		Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
		Vector3 surface_color;
		if (m_color.index() == 0) // RGB color_weight
			surface_color = std::get<Vector3>(m_color);
		else // Texture
		{
			Texture& texture = std::get<Texture>(m_color);
			// Approximate du/dx = (dp/dx) / (dp/du) = radius / dpdu
			Real dp_duv = (length(ray.dpdu), length(ray.dpdv)) / 2.;
			Real footprint = ray.radius / dp_duv;
			Real level = texture.GetLevel(footprint);
			//level = 0;
			//if(level >=4.)
			//	std::cout << "level:" << level << ",radius:" << ray.radius << std::endl;
			surface_color = texture.GetColor(ray.u_coor, ray.v_coor, level);
		}
		Vector3 color_weight = { 0., 0., 0. };
		Real noize_theta = next_pcg32_real<Real>(rng) * c_PI;
		Real noize_phi = next_pcg32_real<Real>(rng) * 2 * c_PI;
		Vector3 sample_in_unit_sphere{ std::sin(noize_theta) * std::cos(noize_phi),
		std::cos(noize_theta),
		std::sin(noize_theta) * std::sin(noize_phi) };



		if (material.index() == 0) // diffuse
		{
			Vector3 sample_direction = normalize(direction);
			Real cos_wo_normal = max(dot(sample_direction, normal), 0.);
			//Real cos_wo_normal = dot(sample_direction, normal);

			color_weight += (surface_color * c_INVPI * cos_wo_normal);
		}
		else if (material.index() == 1) // mirror
		{
			Vector3 sample_direction = normalize(direction);
			Real cos_wo_normal = max(dot(sample_direction, normal), 0.);
			surface_color = surface_color + pow(1. - max(dot(normalize(normal), -normalize(ray.Direction)), 0.), 5.) * (Vector3(1., 1., 1.) - surface_color);
			if(cos_wo_normal > 0)
				color_weight += surface_color;
		}
		else if (material.index() == 2) // plastic
		{
			Plastic& material_plastic = std::get<Plastic>(material);
			Real index_of_refraction = material_plastic.eta;
			Real cos_theta = max(dot(normalize(normal), -normalize(ray.Direction)), 0.);
			//Real cos_theta = max(dot(direction, normal), 0.);
			//std::cout << "cos = " << cos_theta << std::endl;
			Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
			Real F = F_0 + (1. - F_0) * pow(1. - cos_theta, 5);
			//std::cout << "F = " << F << std::endl;
			Vector3 sample_direction = normalize(direction);
			Real cos_wo_normal = max(dot(sample_direction, normal), 0.);
			
			if (is_reflect) //specular
			{
				if (cos_wo_normal > 0)
				{
					color_weight = Vector3(1., 1., 1.);
					//std::cout << color_weight.x << "," << color_weight.y << "," << color_weight.z << std::endl;
				}
				color_weight = Vector3(1., 1., 1.);
				//else
				//	color_weight += surface_color;
				//color_weight += (surface_color * c_INVPI * cos_wo_normal);
			}
			else //diffuse
			{
				color_weight += (surface_color * c_INVPI * cos_wo_normal);
			}
		}
		else if (material.index() == 3) // phong
		{
			Phong& material_phong = std::get<Phong>(material);
			Real alpha = material_phong.exponent;
			if (dot(direction, normal) < 0)
				return 0. * color_weight;
			Real cos_wo_reflect = max(dot(direction, reflect(ray.Direction, normal)), 0.);

			color_weight += (surface_color * (alpha + 1.) * (c_INVPI / 2.) * pow(cos_wo_reflect, alpha));
		}
		else if (material.index() == 4) // blingphong
		{
			BlinnPhong& material_blinnphong = std::get<BlinnPhong>(material);
			if (dot(direction, normal) < 0)
				return 0. * color_weight;
			Real alpha = material_blinnphong.exponent;
			Vector3 sample_direction = direction;
			Vector3 half_vector = normalize(sample_direction - ray.Direction);
			Real cos_norm_half = dot(half_vector, normal);
			Real cos_wo_half = dot(sample_direction, half_vector);
			surface_color = surface_color + pow(1. - cos_wo_half, 5.) * (Vector3(1., 1., 1.) - surface_color);
			color_weight += surface_color * (alpha + 2.) * (c_INVPI / (4. * 2. - pow(2., -alpha / 2.))) * pow(cos_norm_half, alpha);
		}
		else if (material.index() == 5) // blingphongmicro
		{

			BlinnPhongMicrofacet& material_blinnphongMicro = std::get<BlinnPhongMicrofacet>(material);
			if (dot(direction, normal) < 0)
				return 0. * color_weight;
			Real alpha = material_blinnphongMicro.exponent;
			Vector3 sample_direction = direction;
			Vector3 half_vector = normalize(sample_direction - ray.Direction);
			Real cos_norm_half = dot(half_vector, normal);
			Real cos_wo_half = dot(sample_direction, half_vector);
			Real cos_wi_half = dot(-ray.Direction, half_vector);
			Real cos_wi_normal = dot(-ray.Direction, normal);
			Real theta_i_normal = unit_angle(-ray.Direction, normal);
			Real theta_o_normal = unit_angle(sample_direction, normal);
			Real G_i = cos_wi_half > 0 ? GetGeoConfig(theta_i_normal, alpha) : 0.;
			Real G_o = cos_wo_half > 0 ? GetGeoConfig(theta_o_normal, alpha) : 0.;
			Real G_w = G_i * G_o;
			surface_color = surface_color + pow(1. - cos_wo_half, 5.) * (Vector3(1., 1., 1.) - surface_color);
			Real brdf = (alpha + 2.) * (c_INVPI / 8.) * pow(cos_norm_half, alpha) * G_w * (1. / cos_wi_normal);
			if(brdf <= 0)
				return 0. * color_weight;
			color_weight += surface_color * brdf;
		}
		else if (material.index() == 6) // glass
		{
			Glass& material_glass = std::get<Glass>(material);
			Real index_of_refraction = material_glass.eta;

			if (ray.front_face)
			{
				index_of_refraction = 1. / material_glass.eta;
				//normal = -normal;
			}

			double cos_theta = min(dot(-ray.Direction, normal), 1.0);
			double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

			if (index_of_refraction * sin_theta > 1.0 || !is_refract)
			{
				// Must Reflect
				color_weight = surface_color;
			}
			else 
			{
				color_weight = surface_color;
			}
		}
		else if (material.index() == 7)
		{
			// Fuzzed
			//ray.Direction = sample_in_unit_sphere;
			//ray.Direction = normalize(ray.Direction);
			return Vector3(0., 0., 0.);
			/*return color_weight;*/
		}

		return color_weight;
	}
	Real GetPDF(Material material, Ray ray, Vector3 normal, Variables& m_Vars, const Vector3 direction, Scene& scene, BVH& node, pcg32_state& rng, bool is_reflect, bool is_refract)
	{
		Real PDF_value = 0.;

		if (material.index() == 0) // diffuse
		{
			Vector3 sample_direction = direction;
			Real cos_wo_normal = max(dot(sample_direction, normal), 0.);
			Real pdf_inv = c_PI * (1. / cos_wo_normal) / (Real)m_Vars.brdf_samples;
			if (cos_wo_normal <= 0)
				return 0.;
			PDF_value = 1. / pdf_inv;
			return PDF_value;
		}
		else if (material.index() == 1) // mirror
		{
			if(length(direction - reflect(ray.Direction, normal)) == 0)
				PDF_value = 1.;
			return PDF_value;
		}
		else if (material.index() == 2) // plastic
		{
			Plastic& material_plastic = std::get<Plastic>(material);
			Real index_of_refraction = material_plastic.eta;
			Real cos_theta = max(dot(normalize(normal), -normalize(ray.Direction)), 0.);
			//std::cout << "cos = " << cos_theta << std::endl;
			Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
			Real F = F_0 + (1. - F_0) * pow(1. - cos_theta, 5);
			//std::cout << "F = " << F << std::endl;
			//Real p = next_pcg32_real<Real>(rng);
			if (is_reflect) //specular
			{
				Vector3 delta = direction - reflect(ray.Direction, normal);
				if (length(delta) == 0)
					PDF_value = 1.;
				else
					std::cout << "PDF:" << PDF_value << "delta:" << delta.x << "," << delta.y << "," << delta.z << std::endl;
				return PDF_value;
			}
			//Vector3 delta = direction - reflect(ray.Direction, normal);
			//if (length(delta) == 0)
			//	PDF_value = 1.;
			else //diffuse
			{

				Vector3 sample_direction = direction;
				Real cos_wo_normal = max(dot(sample_direction, normal), 0.);
				Real pdf_inv = c_PI * (1. / cos_wo_normal) / (Real)m_Vars.brdf_samples;
				//if (cos_wo_normal < 0)
				//{
				//	PDF_value = 0.;
				//	return PDF_value;
				//}
				PDF_value = 1. / pdf_inv;
				//PDF_value = 1.;
				return PDF_value;
			}
		}
		else if (material.index() == 3) // phong
		{
			Phong& material_phong = std::get<Phong>(material);
			Vector3 sample_direction = direction;
			Real cos_wo_normal = max(dot(sample_direction, normal), 0.);
			Real alpha = material_phong.exponent;
			if (cos_wo_normal < 0)
			{
				PDF_value = 0.;
				return PDF_value;
			}
			Real pdf_inv = 2 * c_PI * (1. / (alpha + 1.)) * (1. / pow(cos_wo_normal, alpha)) / (Real)m_Vars.brdf_samples;
			PDF_value = 1. / pdf_inv;
			return PDF_value;
		}
		else if (material.index() == 4) // blingphong
		{
			BlinnPhong& material_blinnphong = std::get<BlinnPhong>(material);
			Vector3 sample_direction = direction;
			Vector3 half_vector = normalize(sample_direction - ray.Direction);
			Real cos_norm_half = dot(half_vector, normal);
			Real cos_wo_half = dot(sample_direction, half_vector);
			Real cos_wo_normal = max(dot(sample_direction, normal), 0.);
			Real alpha = material_blinnphong.exponent;
			if (cos_wo_normal < 0)
			{
				PDF_value = 0.;
				return PDF_value;
			}
			Real pdf_inv = 8 * c_PI * cos_wo_half *
				(1. / ((alpha + 1.) * pow(cos_norm_half, alpha))) / (Real)m_Vars.brdf_samples;
			PDF_value = 1. / pdf_inv;
			return PDF_value;
		}
		else if (material.index() == 5) // blingphongmicro
		{

			BlinnPhongMicrofacet& material_blinnphongMicro = std::get<BlinnPhongMicrofacet>(material);
			Vector3 sample_direction = direction;
			Vector3 half_vector = normalize(sample_direction - ray.Direction);
			Real cos_norm_half = dot(half_vector, normal);
			Real cos_wo_half = dot(sample_direction, half_vector);
			Real cos_wo_normal = max(dot(sample_direction, normal), 0.);
			Real alpha = material_blinnphongMicro.exponent;
			if (cos_wo_normal < 0)
			{
				PDF_value = 0.;
				return PDF_value;
			}
			Real pdf_inv = 8 * c_PI * cos_wo_half *
				(1. / ((alpha + 1.) * pow(cos_norm_half, alpha))) / (Real)m_Vars.brdf_samples;
			PDF_value = 1. / pdf_inv;
			return PDF_value;
		}
		else if (material.index() == 6) // glass
		{
			Glass& material_glass = std::get<Glass>(material);
			Real index_of_refraction = material_glass.eta;

			if (ray.front_face)
			{
				index_of_refraction = 1. / material_glass.eta;
				//normal = -normal;
			}

			double cos_theta = min(dot(-ray.Direction, normal), 1.);
			double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

			Real kr = reflectance(cos_theta, index_of_refraction);
			Real kt = 1 - kr;

			Vector3 refracted_direction = normalize(refract(ray.Direction, normal, index_of_refraction));
			// Can Refract
			Vector3 reflect_direction = reflect(ray.Direction, normal);

			if (index_of_refraction * sin_theta > 1.0 || !is_refract)
			{
				// Must Reflect
				Vector3 reflect_direction = reflect(ray.Direction, normal);
				if (length(reflect_direction - direction) == 0)
					return 1.;
			}
			else 
			{
				if (length(refracted_direction - direction) == 0)
					return 1.;
			}
			return 0.;

		}
		else if (material.index() == 7)
		{
			// Fuzzed

			return 1.;
			/*return color_weight;*/
		}
		return PDF_value;
	};
	Vector3 SampleDirection(Material material, Ray ray, Vector3 normal, Variables& m_Vars, Scene& scene, BVH& node, pcg32_state& rng, bool* is_reflect, bool* is_refract)
	{
		Real noize_theta = next_pcg32_real<Real>(rng) * c_PI;
		Real noize_phi = next_pcg32_real<Real>(rng) * 2 * c_PI;
		Vector3 sample_in_unit_sphere{ std::sin(noize_theta) * std::cos(noize_phi),
		std::cos(noize_theta),
		std::sin(noize_theta) * std::sin(noize_phi) };

		if (material.index() == 0) // diffuse
		{
			Real u1 = next_pcg32_real<Real>(rng);
			Real u2 = next_pcg32_real<Real>(rng);

			Real theta = acos(1. - 2. * u1) / 2.;
			Real phi = 2. * c_PI * u2;
			Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
			Vector3 z_axis = normal;
			Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
			Vector3 y_axis = cross(z_axis, x_axis);
			Vector3 sample_direction = sample_direction_ori.x * x_axis +
				sample_direction_ori.y * y_axis +
				sample_direction_ori.z * z_axis;
			return normalize(sample_direction);
		}
		else if (material.index() == 1) // mirror
		{
			return reflect(ray.Direction, normal);
		}
		else if (material.index() == 2) // plastic
		{
			Plastic& material_plastic = std::get<Plastic>(material);
			Real index_of_refraction = material_plastic.eta;
			Real cos_theta = max(dot(normalize(normal), -normalize(ray.Direction)), 0.);
			//std::cout << "cos = " << cos_theta << std::endl;
			Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
			Real F = F_0 + (1. - F_0) * pow(1. - cos_theta, 5);
			//std::cout << "F = " << F << std::endl;

			Real random_f = next_pcg32_real<Real>(rng);
			if (random_f <= F) //specular
			{
				*is_reflect = true;
				return reflect(ray.Direction, normal);
			}
			else //diffuse
			{
				Real u1 = next_pcg32_real<Real>(rng);
				Real u2 = next_pcg32_real<Real>(rng);

				Real theta = acos(1. - 2. * u1) / 2.;
				Real phi = 2. * c_PI * u2;
				Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
				Vector3 z_axis = normal;
				Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
				Vector3 y_axis = cross(z_axis, x_axis);
				Vector3 sample_direction = sample_direction_ori.x * x_axis +
					sample_direction_ori.y * y_axis +
					sample_direction_ori.z * z_axis;
				return normalize(sample_direction);
			}
		}
		else if (material.index() == 3) // phong
		{
			Phong& material_phong = std::get<Phong>(material);

			Real u1 = next_pcg32_real<Real>(rng);
			Real u2 = next_pcg32_real<Real>(rng);
			Real alpha = material_phong.exponent;

			Real theta = acos(pow(1. - u1, 1. / (alpha + 1)));
			Real phi = 2. * c_PI * u2;
			Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
			Vector3 z_axis = reflect(ray.Direction, normal);
			Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
			Vector3 y_axis = cross(z_axis, x_axis);
			Vector3 sample_direction = sample_direction_ori.x * x_axis +
				sample_direction_ori.y * y_axis +
				sample_direction_ori.z * z_axis;
			return normalize(sample_direction);
		}
		else if (material.index() == 4) // blingphong
		{
			BlinnPhong& material_blinnphong = std::get<BlinnPhong>(material);


			Real u1 = next_pcg32_real<Real>(rng);
			Real u2 = next_pcg32_real<Real>(rng);
			Real alpha = material_blinnphong.exponent;

			Real theta = acos(pow(1. - u1, 1. / (alpha + 1)));
			Real phi = 2. * c_PI * u2;
			Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
			Vector3 z_axis = normal;
			Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
			Vector3 y_axis = cross(z_axis, x_axis);
			Vector3 sample_direction = sample_direction_ori.x * x_axis +
				sample_direction_ori.y * y_axis +
				sample_direction_ori.z * z_axis;
			sample_direction = normalize(sample_direction);
			Vector3 half_vector = sample_direction;
			Vector3 w_o = reflect(ray.Direction, half_vector);
			return w_o;

		}
		else if (material.index() == 5) // blingphongmicro
		{

			BlinnPhongMicrofacet& material_blinnphongMicro = std::get<BlinnPhongMicrofacet>(material);

			Real u1 = next_pcg32_real<Real>(rng);
			Real u2 = next_pcg32_real<Real>(rng);
			Real alpha = material_blinnphongMicro.exponent;

			Real theta = acos(pow(1. - u1, 1. / (alpha + 1)));
			Real phi = 2. * c_PI * u2;
			Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
			Vector3 z_axis = normal;
			Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
			Vector3 y_axis = cross(z_axis, x_axis);
			Vector3 sample_direction = sample_direction_ori.x * x_axis +
				sample_direction_ori.y * y_axis +
				sample_direction_ori.z * z_axis;
			sample_direction = normalize(sample_direction);
			Vector3 half_vector = sample_direction;
			Vector3 w_o = reflect(ray.Direction, half_vector);
			return w_o;
		}
		else if (material.index() == 6) // glass
		{
			Glass& material_glass = std::get<Glass>(material);
			Real index_of_refraction = material_glass.eta;

			if (ray.front_face)
			{
				index_of_refraction = 1. / material_glass.eta;
				//normal = -normal;
			}

			double cos_theta = min(dot(-ray.Direction, normal), 1.0);
			double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

			Real kr = reflectance(cos_theta, index_of_refraction);
			Real kt = 1 - kr;

			Vector3 refracted_direction = normalize(refract(ray.Direction, normal, index_of_refraction));
			// Can Refract
			Vector3 reflect_direction = reflect(ray.Direction, normal);
			Real random_p = next_pcg32_real<Real>(rng);

			if (index_of_refraction * sin_theta > 1.0 || random_p < kr)
			{
				// Must Reflect
				reflect_direction = reflect(ray.Direction, normal);
				*is_refract = false;
				return reflect_direction;
			}
			else 
			{
				*is_refract = true;
				return refracted_direction;
			}

		}
		else if (material.index() == 7)
		{
			// Fuzzed
			return normalize(sample_in_unit_sphere);
			/*return color_weight;*/
		}
	}

};

class Light_Sampler : public Sampler
{
public:
	Vector3 GetBRDF(Material material, Ray ray, Vector3 normal, Variables& m_Vars, const Vector3 direction, Scene& scene, BVH& node, pcg32_state& rng, bool is_reflect, bool is_refract)
	{
		BRDF_Sampler sampler;
		return sampler.GetBRDF(material, ray, normal, m_Vars, direction, scene, node, rng, is_reflect, is_refract);
		//return visibility * sampler.GetBRDF(material, ray, normal, m_Vars, direction, scene, node, rng);
	}
	Real GetPDF(Material material, Ray ray, Vector3 normal, Variables& m_Vars, const Vector3 direction, Scene& scene, BVH& node, pcg32_state& rng, bool is_reflect, bool is_refract)
	{
		Real PDF_value = 0.;
		Ray shadow_ray = { ray.Origin, normalize(direction) };
		if (material.index() == 1) // mirror
		{
			if (length(direction - reflect(ray.Direction, normal)) == 0)
			{
				PDF_value = 1.;
			}
			return PDF_value;
		}
		else if (material.index() == 2) // plastic
		{
			//BRDF_Sampler sampler;
			//return sampler.GetPDF(material, ray, normal, m_Vars, direction, scene, node, rng, is_reflect);
			Plastic& material_plastic = std::get<Plastic>(material);
			Real index_of_refraction = material_plastic.eta;
			Real cos_theta = max(dot(normalize(normal), -normalize(ray.Direction)), 0.);
			//std::cout << "cos = " << cos_theta << std::endl;
			Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
			Real F = F_0 + (1. - F_0) * pow(1. - cos_theta, 5);
			//std::cout << "F = " << F << std::endl;
			//Real p = next_pcg32_real<Real>(rng);
			if (is_reflect) //specular
			{
				Vector3 delta = direction - reflect(ray.Direction, normal);
				if (length(delta) == 0)
					PDF_value = 1.;
				return PDF_value;
			}
		}
		else if (material.index() == 6) // glass
		{
			Glass& material_glass = std::get<Glass>(material);
			Real index_of_refraction = material_glass.eta;

			if (ray.front_face)
			{
				index_of_refraction = 1. / material_glass.eta;
				//normal = -normal;
			}

			double cos_theta = min(dot(-ray.Direction, normal), 1.);
			double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

			Real kr = reflectance(cos_theta, index_of_refraction);
			Real kt = 1 - kr;

			Vector3 refracted_direction = normalize(refract(ray.Direction, normal, index_of_refraction));
			// Can Refract
			Vector3 reflect_direction = reflect(ray.Direction, normal);

			if (index_of_refraction * sin_theta > 1.0 || !is_refract)
			{
				// Must Reflect
				Vector3 reflect_direction = reflect(ray.Direction, normal);
				if (length(reflect_direction - direction) == 0)
					return 1.;
			}
			else
			{
				if (length(refracted_direction - direction) == 0)
					return 1.;
			}
		}

		if (!Renderer::FindNearstIntersection_BVH(shadow_ray, scene, node, 1e-4, m_Vars.t_max, rng))
		{
			bool exist_envmap = false;
			for (ParsedLight light : scene.lights)
			{
				if (light.index() == 2) // env_map
				{
					exist_envmap = true;
					ParsedEnvMap& envmap = std::get<ParsedEnvMap>(light);
					Vector4 direc_light_4 = envmap.world_to_light * Vector4(normalize(direction), 0.);
					Vector3 direc_light = normalize(Vector3(direc_light_4.x, direc_light_4.y, direc_light_4.z));
					Vector2 uv_real = envmap.GetUV(direc_light);
					Vector2i uv_int = { (int)(uv_real[0] * (Real)envmap.texture.width),
										(int)(uv_real[1] * (Real)envmap.texture.height) };
					Real pdf = envmap.dist_2d.GetPDF(uv_int) / envmap.dist_2d.marginal_rows->funcIntegral;
					pdf *= (Real)envmap.texture.width * (Real)envmap.texture.height;
					//std::cout << pdf << std::endl;
					Real sin_theta = sin(uv_real[1] * c_PI);
					if (sin_theta < 0.)
						return 0.;
					Real pdf_wo = pdf / (2. * c_PI * c_PI * sin_theta);
					PDF_value = pdf_wo;
				}
			}
			
			if(!exist_envmap)
				return 0.;
		}
		else
		{
			int shadow_hit_area_light_id = std::visit([&](auto shape) {return shape.area_light_id; }, *shadow_ray.object);
			int visibility = 1;
			//if (shadow_hit_area_light_id == -1) // hit object instead of light
			//	return 0.;
			 // is area light
			//{
			//	if (shadow_ray.object->index() == 0) // sphere
			//	{
			//		ParsedSphere& light_sphere = std::get<ParsedSphere>(*shadow_ray.object);
			//		//Ray ray_copy = { ray.Origin, normalize(direction) };
			//		//ray_copy.FindIntersection(light_sphere, translate(light_sphere.position), m_Vars.t_min, m_Vars.t_max);
			//		Real dc = length(light_sphere.position - ray.Origin);
			//		Real cos_theta_max = sqrt(max(0., (light_sphere.radius * light_sphere.radius) / (dc * dc)));
			//		Vector3 hit_p = shadow_ray.Origin + shadow_ray.distance * shadow_ray.Direction;
			//		Vector4 normal_original = GetNormalByHitPoint(hit_p - light_sphere.position, light_sphere);
			//		Vector4 normal_transformed_4 = Vector4(transpose(inverse(
			//			translate(normalize(Vector3(light_sphere.position.x, light_sphere.position.y, light_sphere.position.z))))) * normal_original);
			//		Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
			//		Real cos_light = max(dot(-shadow_ray.Direction, normal_transformed), 0.);
			//		if (cos_light <= 0)
			//			PDF_value = 0.;
			//		else
			//			PDF_value = (shadow_ray.distance * shadow_ray.distance) / (cos_light * 2. * (1. - cos_theta_max) * c_PI * light_sphere.radius * light_sphere.radius);
			//	}
			//	else // trianglemesh
			//	{
			//		Triangle light_triangle = std::get<Triangle>(*shadow_ray.object);
			//		//Triangle light_triangle = std::get<Triangle>(*shadow_ray.object);
			//		//Ray ray_copy = { ray.Origin, normalize(direction) };
			//		//if (ray_copy.FindIntersection(light_triangle, translate(Vector3(0., 0., 0.)), m_Vars.t_min, m_Vars.t_max))
			//		//{
			//		Vector3 hit_p = shadow_ray.Origin + shadow_ray.distance * shadow_ray.Direction;
			//		//Vector3 normal_1 = light_triangle.mesh->normals[light_triangle.mesh->indices[light_triangle.index][0]];
			//		//Vector3 normal_2 = light_triangle.mesh->normals[light_triangle.mesh->indices[light_triangle.index][1]];
			//		//Vector3 normal_3 = light_triangle.mesh->normals[light_triangle.mesh->indices[light_triangle.index][2]];
			//		//Vector3 normal_p = normalize(normal_1 / 3. + normal_2 / 3. + normal_3 / 3.);
			//		//Vector3 normal_transformed = normal_p;
			//		Vector4 normal_original = GetNormalByHitPoint(hit_p, light_triangle);
			//		Vector4 normal_transformed_4 = normal_original;
			//		Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
			//		Real cos_light = max(dot(-shadow_ray.Direction, normal_transformed), 0.);
			//		if (cos_light <= 0)
			//			PDF_value = 0.;
			//		else
			//			PDF_value = (shadow_ray.distance * shadow_ray.distance) / (cos_light * 2. * GetAreaByShape(light_triangle));
			//		//}
			//	}
			//}


			//ParsedLight hit_light = scene.lights[shadow_hit_area_light_id];
			int index = 0;
			bool hit_envmap = true;
			for(ParsedLight light : scene.lights)
			{
				if(shadow_hit_area_light_id != index++)
				{
					if(shadow_hit_area_light_id != -1)
						continue;
				}
				if (light.index() == 0) // is PointLight
				{
					PDF_value = 1.;
				}
				else if (light.index() == 1) // is area light
				{
					ParsedDiffuseAreaLight area_light = std::get<ParsedDiffuseAreaLight>(light);
					if (scene.src_shapes[area_light.shape_id]->index() == 0) // sphere
					{
						ParsedSphere& light_sphere = std::get<ParsedSphere>(*scene.src_shapes[area_light.shape_id]);
						Ray ray_copy = { ray.Origin, normalize(direction) };
						if(ray_copy.FindIntersection(light_sphere, translate(light_sphere.position), m_Vars.t_min, m_Vars.t_max))
						{
							hit_envmap = false;
							Real dc = length(light_sphere.position - ray.Origin);
							Real cos_theta_max = sqrt(max(0., (light_sphere.radius * light_sphere.radius) / (dc * dc)));
							Vector3 hit_p = ray_copy.Origin + ray_copy.distance * ray_copy.Direction;
							Vector4 normal_original = GetNormalByHitPoint(hit_p - light_sphere.position, light_sphere);
							Vector4 normal_transformed_4 = Vector4(transpose(inverse(
								translate(normalize(Vector3(light_sphere.position.x, light_sphere.position.y, light_sphere.position.z))))) * normal_original);
							Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
							Real cos_light = max(dot(-ray_copy.Direction, normal_transformed),0.);
							if (cos_light <= 0)
								PDF_value = 0.;
							else
								PDF_value = (ray_copy.distance * ray_copy.distance) / (cos_light * 2. * (1. - cos_theta_max) * c_PI * light_sphere.radius * light_sphere.radius);
						}
						//std::cout << "pdf:" << PDF_value  << ",cos_max:" << cos_theta_max << ",dc:" << dc << std::endl;
					}
					else // trianglemesh
					{
						ParsedTriangleMesh light_mesh = std::get<ParsedTriangleMesh>(*scene.src_shapes[area_light.shape_id]);
						for (int i = 0; i < light_mesh.indices.size(); i++)
						{
							Triangle light_triangle = { &light_mesh, i, light_mesh.area_light_id };
							//Triangle light_triangle = std::get<Triangle>(*shadow_ray.object);
							Ray ray_copy = { ray.Origin, normalize(direction) };
							if(ray_copy.FindIntersection(light_triangle, translate(Vector3(0., 0., 0.)), m_Vars.t_min, m_Vars.t_max))
							{
								hit_envmap = false;
								Vector3 hit_p = ray_copy.Origin + ray_copy.distance * ray_copy.Direction;
								//Vector3 normal_1 = light_triangle.mesh->normals[light_triangle.mesh->indices[light_triangle.index][0]];
								//Vector3 normal_2 = light_triangle.mesh->normals[light_triangle.mesh->indices[light_triangle.index][1]];
								//Vector3 normal_3 = light_triangle.mesh->normals[light_triangle.mesh->indices[light_triangle.index][2]];
								//Vector3 normal_p = normalize(normal_1 / 3. + normal_2 / 3.+ normal_3 / 3.);
								//Vector3 normal_transformed = normal_p;
								Vector4 normal_original = GetNormalByHitPoint(hit_p, light_triangle);
								Vector4 normal_transformed_4 = normal_original;
								Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
								Real cos_light = max(dot(-ray_copy.Direction, normal_transformed), 0.);
								if (cos_light <= 0)
									PDF_value = 0.;
								else
									PDF_value = (ray_copy.distance * ray_copy.distance) / (cos_light * light_mesh.indices.size() * GetAreaByShape(light_triangle));
							}
						}
					}
				}
				
			}

			if (hit_envmap)
			{
				for(ParsedLight light : scene.lights)
				{
					if (light.index() == 2) // env_map
					{
						ParsedEnvMap& envmap = std::get<ParsedEnvMap>(light);
						Vector4 direc_light_4 = envmap.world_to_light * Vector4(normalize(direction), 0.);
						Vector3 direc_light = normalize(Vector3(direc_light_4.x, direc_light_4.y, direc_light_4.z));
						Vector2 uv_real = envmap.GetUV(direc_light);
						Vector2i uv_int = { (int)(uv_real[0] * (Real)envmap.texture.width),
											(int)(uv_real[1] * (Real)envmap.texture.height) };
						Real pdf = envmap.dist_2d.GetPDF(uv_int) / envmap.dist_2d.marginal_rows->funcIntegral;
						pdf *= (Real)envmap.texture.width * (Real)envmap.texture.height;
						Real sin_theta = sin(uv_real[1] * c_PI);
						if (sin_theta < 0.)
							return 0.;
						Real pdf_wo = pdf / (2. * c_PI * c_PI * sin_theta);
						PDF_value = pdf_wo;
					}
				}
			}
		}
		return PDF_value / scene.lights.size();
	}
	Vector3 SampleDirection(Material material, Ray ray, Vector3 normal, Variables& m_Vars, Scene& scene, BVH& node, pcg32_state& rng, bool* is_reflect, bool* is_refract)
	{
		if (material.index() == 1) // mirror
		{
			return reflect(ray.Direction, normal);
		}
		else if (material.index() == 2) // plastic
		{
			//BRDF_Sampler sampler;
			//Vector3 direction = sampler.SampleDirection(material, ray, normal, m_Vars, scene, node, rng, is_reflect);
			//return direction;
			Plastic& material_plastic = std::get<Plastic>(material);
			Real index_of_refraction = material_plastic.eta;
			Real cos_theta = max(dot(normalize(normal), -normalize(ray.Direction)), 0.);
			//std::cout << "cos = " << cos_theta << std::endl;
			Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
			Real F = F_0 + (1. - F_0) * pow(1. - cos_theta, 5);
			//std::cout << "F = " << F << std::endl;

			Real random_f = next_pcg32_real<Real>(rng);
			if (random_f <= F) //specular
			{
				*is_reflect = true;
				return reflect(ray.Direction, normal);
			}
		}
		else if (material.index() == 6)//glass
		{
			BRDF_Sampler sampler;
			return sampler.SampleDirection(material, ray, normal, m_Vars, scene, node, rng, is_reflect, is_refract);
		}
		Real random_p = next_pcg32_real<Real>(rng);

		//std::vector<Real> energys;
		//int max_index = 0;
		//int current_index = 0;
		//Real max_energy = 0.;
		//for (ParsedLight light : scene.lights)
		//{
		//	if (light.index() == 0) // is PointLight
		//	{
		//		;
		//	}
		//	else // is area light
		//	{
		//		ParsedDiffuseAreaLight area_light = std::get<ParsedDiffuseAreaLight>(light);
		//		if (scene.src_shapes[area_light.shape_id]->index() == 0) // sphere
		//		{
		//			ParsedSphere& light_sphere = std::get<ParsedSphere>(*scene.src_shapes[area_light.shape_id]);
		//			Real dc = length(light_sphere.position - ray.Origin);
		//			Vector3 dir = normalize(light_sphere.position - ray.Origin);
		//			Real cos_theta_max = sqrt(max(0., (light_sphere.radius * light_sphere.radius) / (dc * dc)));
		//			Real area_m_l = 2. * (1. - cos_theta_max) * c_PI * light_sphere.radius * light_sphere.radius * length(area_light.radiance);
		//			Vector3 brdf = GetBRDF(material, ray, normal, m_Vars, dir, scene, node, rng, is_reflect);
		//			Real energy =  length(brdf) * area_m_l / (dc * dc);
		//			energys.push_back(energy);
		//			//std::cout << "pdf:" << PDF_value  << ",cos_max:" << cos_theta_max << ",dc:" << dc << std::endl;
		//		}
		//		else // trianglemesh
		//		{
		//			ParsedTriangleMesh light_mesh = std::get<ParsedTriangleMesh>(*scene.src_shapes[area_light.shape_id]);
		//			Real mesh_area_m_l = 0.;
		//			Real energy = 0.;
		//			for (int i = 0; i < light_mesh.indices.size(); i++)
		//			{
		//				Triangle light_triangle = { &light_mesh, i, light_mesh.area_light_id };
		//				Vector3 center = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][0]] +
		//					light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][1]] +
		//					light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][2]];
		//				center *= 1. / 3.;
		//				Real distance = length(ray.Origin - center);
		//				mesh_area_m_l += GetAreaByShape(light_triangle) * length(area_light.radiance);
		//				energy += dot(normalize(center - ray.Origin), normal)  * mesh_area_m_l / pow(distance, 2.);
		//			}
		//			energys.push_back(energy);
		//		}
		//		if (max_energy < energys[energys.size()-1])
		//		{
		//			max_energy = energys[energys.size() - 1];
		//			max_index = current_index;
		//		}
		//	}
		//	current_index++;
		//}

		std::vector<Real> lights_pdf;
		for(int i =0; i < scene.lights.size();i++)
		{
			lights_pdf.push_back(1.);
		}
		Distribution1D lights_distribution(lights_pdf);
		int random_light_index = lights_distribution.sample(rng);
		//std::cout << random_light_index << std::endl;
		ParsedLight light_random = scene.lights[random_light_index];
		//ParsedLight light_max = scene.lights[max_index];
		ParsedLight& light_target = light_random;


		//choose max light;
		//if (random_p < 0.)
		//	light_target = light_random;
		//else
		//	light_target = light_max;

		//choose from distributed inverse sample
		//Real accumulate_p = 0.;
		//Real totaly_energy  = std::accumulate(energys.begin(), energys.end(), 0.);
		//int sample_index = 0;
		//for(int i = 0; i < energys.size(); i++)
		//{
		//	accumulate_p += energys[i] / totaly_energy;
		//	if(random_p <= accumulate_p)
		//	{
		//		sample_index = i;
		//		break;
		//	}
		//}
		//light_target = scene.lights[sample_index];

		std::vector<Real> visibility;
		std::vector<Vector3> position;
		Vector3 light_direction = Vector3(0., 0., 0.);


		if (light_random.index() == 0) // is PointLight
		{
			ParsedPointLight point_light = std::get<ParsedPointLight>(light_random);
			position.push_back(point_light.position);
			light_direction = ray.Origin - point_light.position;
			Ray shadow_ray = { ray.Origin, -normalize(light_direction) };
			if (!Renderer::FindNearstIntersection_BVH(shadow_ray, scene, node, 1e-4, length(light_direction), rng))
				visibility.push_back(1.);
			else
				visibility.push_back(0);
			return normalize(point_light.position - ray.Origin);
		}
		else if(light_random.index() == 1)// is area light
		{
			ParsedDiffuseAreaLight area_light = std::get<ParsedDiffuseAreaLight>(light_target);
			if (scene.src_shapes[area_light.shape_id]->index() == 0) // sphere
			{
				int sqrt_N = sqrt(m_Vars.area_light_samples);
				Real u1;
				Real u2;
				for (int i = 0; i < sqrt_N; i++) {
					for (int j = 0; j < sqrt_N; j++) {
						Real u1_ori = next_pcg32_real<Real>(rng);
						Real u2_ori = next_pcg32_real<Real>(rng);
						u1 = (i + u1_ori) / sqrt_N;
						u2 = (j + u2_ori) / sqrt_N;

						ParsedSphere& light_sphere = std::get<ParsedSphere>(*scene.src_shapes[area_light.shape_id]);
						Real dc = length(light_sphere.position - ray.Origin);
						Real cos_theta_max = sqrt(max(0., (light_sphere.radius * light_sphere.radius) / (dc * dc)));

						Real theta = acos(1. + (cos_theta_max - 1.) * u1);
						//std::cout << "theta:" << theta << std::endl;
						//Real theta = acos(1. - 2. * u1);
						Real phi = 2. * c_PI * u2;
						Real r = light_sphere.radius;
						Real cos_theta = cos(theta);
						Real sin_theta = sin(theta);
						Vector3 offset = { sin_theta * cos(phi), sin_theta * sin(phi), cos_theta };
						Vector3 z_axis = normalize(ray.Origin - light_sphere.position);
						Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
						Vector3 y_axis = cross(z_axis, x_axis);
						Vector3 sample_direction = offset.x * x_axis +
							offset.y * y_axis +
							offset.z * z_axis;
						sample_direction = normalize(sample_direction);
						offset = sample_direction * light_sphere.radius;
						Vector3 hit_p = light_sphere.position + offset;
						position.push_back(hit_p);
						//light_direction = ray.Origin - hit_p;
						//Ray shadow_ray = { ray.Origin, -normalize(light_direction) };
						//if (!Renderer::FindNearstIntersection_BVH(shadow_ray, scene, node, 1e-4, length(light_direction), rng))
						//	visibility.push_back(1.);
						//else
						//{
						//	int shadow_hit_area_light_id = std::visit([&](auto shape) {return shape.area_light_id; }, *shadow_ray.object);
						//	if (shadow_hit_area_light_id == light_sphere.area_light_id)
						//		visibility.push_back(1.);
						//	else
						//		visibility.push_back(0);
						//}

						//light_direction = normalize(light_direction);
						//Vector4 normal_original = GetNormalByHitPoint(offset, light_sphere);
						//Vector4 normal_transformed_4 = Vector4(transpose(inverse(
						//	translate(normalize(Vector3(light_sphere.position.x, light_sphere.position.y, light_sphere.position.z))))) * normal_original);
						//Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
						//Real cos_light = max(dot(light_direction, normal_transformed), 0.);
					}
				}
			}
			else // trianglemesh
			{
				ParsedTriangleMesh& light_mesh = std::get<ParsedTriangleMesh>(*scene.src_shapes[area_light.shape_id]);
				for (int i = 0; i < light_mesh.indices.size(); i++)
				{
					if (light_mesh.indices.size() > 100)
					{
						if (i % light_mesh.indices.size() != 0)
							continue;
					}
					int sqrt_N = sqrt(m_Vars.area_light_samples);
					Real u1;
					Real u2;
					for (int k = 0; k < sqrt_N; k++) {
						for (int j = 0; j < sqrt_N; j++) {
							Real u1_ori = next_pcg32_real<Real>(rng);
							Real u2_ori = next_pcg32_real<Real>(rng);
							u1 = (k + u1_ori) / sqrt_N;
							u2 = (j + u2_ori) / sqrt_N;
							//std::cout << "u1:" << u1 << ",u2:" << u2 << std::endl;
							Triangle light_triangle = { &light_mesh, i, light_mesh.area_light_id };
							Real b1 = 1. - sqrt(u1);
							Real b2 = u2 * sqrt(u1);
							//std::cout << "b1:" << b1 << ",b2:" << b2 << std::endl;
							Vector3 p1 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][0]];
							Vector3 p2 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][1]];
							Vector3 p3 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][2]];
							Vector3 hit_p = (1. - b1 - b2) * p1 + b1 * p2 + b2 * p3;
							position.push_back(hit_p);
							//light_direction = ray.Origin - hit_p;
							//Ray shadow_ray = { ray.Origin, -normalize(light_direction) };
							//if (!Renderer::FindNearstIntersection_BVH(shadow_ray, scene, node, 1e-4, length(light_direction), rng))
							//	visibility.push_back(1.);
							//else
							//{
							//	int shadow_hit_area_light_id = std::visit([&](auto shape) {return shape.area_light_id; }, *shadow_ray.object);
							//	if (shadow_hit_area_light_id == light_mesh.area_light_id)
							//		visibility.push_back(1.);
							//	else
							//		visibility.push_back(0);
							//}

							//light_direction = normalize(light_direction);
							//Vector4 normal_original = GetNormalByHitPoint(hit_p, light_triangle);
							//Vector4 normal_transformed_4 = normal_original;
							//Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
							//Real cos_light = max(dot(light_direction, normal_transformed), 0.);

						}
					}
				}
			}
			int random_light_area_index = (int)(next_pcg32_real<Real>(rng) * (Real)position.size());
			//std::cout << random_light_area_index << std::endl;
			return normalize(position[random_light_area_index] - ray.Origin);
		}
		else if(light_random.index() == 2)
		{
			ParsedEnvMap& envmap = std::get<ParsedEnvMap>(light_random);
			Vector2i uv_int = envmap.dist_2d.sample(rng);
			Vector2 uv_real = { (Real)uv_int[0] / (Real)envmap.texture.width,
								(Real)uv_int[1] / (Real)envmap.texture.height };
			Real theta = uv_real[1] * c_PI;
			Real phi = uv_real[0] * 2. * c_PI;

			Vector3 direc_light = { -cos(phi) * sin(theta),
									cos(theta),
									sin(phi) * sin(theta) };
			Vector4 direc_world_4 = envmap.light_to_world * Vector4(direc_light, 0.);
			Vector3 direc_world = normalize(Vector3(direc_world_4.x, direc_world_4.y, direc_world_4.z));

			return direc_world;

		}



		//	Vector3 color_from_light = Vector3(0., 0., 0.);
		//	Vector3 normal = normalize(normal);
		//	Real largest_weight = 0.;
		//	int largest_index = 0;

		//	if (dot(ray_direction_ori, normal) > 0)
		//	{
		//		normal = -normal;
		//	}

		//	ray.Direction = -light_direction;
		//	for (int i = 0; i < position.size(); i++)
		//	{
		//		light_direction = normalize(ray.Origin - position[i]);
		//		Real weight = visibility[i];
		//		color_from_light += surface_color * weight * intensity[i];
		//		if (largest_weight < weight)
		//		{
		//			largest_weight = weight;
		//			largest_index = i;
		//		}
		//	}

		//	if (pdf_and_cos[largest_index] > 0 && visibility[largest_index] == 1)
		//	{
		//		ray.Direction = -normalize(HitPoint - position[largest_index]);
		//	}
		//	if (pdf_and_cos[largest_index] > 0 && dot(-light_direction, normal) > 0)
		//		PDF_value.push_back(visibility[largest_index] * (vars.attenuation_const + vars.attenuation_linear * d[largest_index] + vars.attenuation_quadratic * d[largest_index] * d[largest_index]) / pdf_and_cos[largest_index]);
		//	color_weight.push_back(largest_weight * surface_color);

		//	color += color_from_light;

	}

};

class One_Mix_Sampler : public Sampler
{
public:
	Vector3 GetBRDF(Material material, Ray ray, Vector3 normal, Variables& m_Vars, const Vector3 direction, Scene& scene, BVH& node, pcg32_state& rng, bool is_reflect, bool is_refract)
	{
		return BRDF_sampler.GetBRDF(material, ray, normal, m_Vars, direction,scene, node, rng, is_reflect, is_refract);
	}
	Real GetPDF(Material material, Ray ray, Vector3 normal, Variables& m_Vars, const Vector3 direction, Scene& scene, BVH& node, pcg32_state& rng, bool is_reflect, bool is_refract)
	{
		//std::cout << "brdf_pdf:" << BRDF_sampler.GetPDF(material, ray, normal, m_Vars, direction, scene, node, rng) << ", light_pdf:" << Light_sampler.GetPDF(material, ray, normal, m_Vars, direction, scene, node, rng) << std::endl;
		return (BRDF_sampler.GetPDF(material, ray, normal, m_Vars, direction, scene, node, rng, is_reflect, is_refract)
			+ Light_sampler.GetPDF(material, ray, normal, m_Vars, direction, scene, node, rng, is_reflect, is_refract)) / 2.;
	}
	Vector3 SampleDirection(Material material, Ray ray, Vector3 normal, Variables& m_Vars, Scene& scene, BVH& node, pcg32_state& rng, bool* is_reflect, bool* is_refract)
	{
		Real random_p = next_pcg32_real<Real>(rng);
		p = random_p;
		if (random_p < 0.5)
		{
			Vector3 direction = BRDF_sampler.SampleDirection(material, ray, normal, m_Vars, scene, node, rng, is_reflect, is_refract);
			return direction;
		}
		else
		{
			Vector3 direction = Light_sampler.SampleDirection(material, ray, normal, m_Vars, scene, node, rng, is_reflect, is_refract);
			return direction;
		}
	}
private:
	BRDF_Sampler BRDF_sampler;
	Light_Sampler Light_sampler;
	Real p;
public:
	bool is_plastic_reflect;
};

void Renderer::Render(CameraUnion& camera, Variables& vars, Scene& scene,
                      Vector3(Renderer::*miss)(const Ray&, Scene&, Variables&, int),
                      Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&))
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
	Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&))
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
	Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&))
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


	if(m_Vars.parallel_counts_bvh > 1 && src_shapes.size() > 512)
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

void Renderer::Render_BVH_Path(CameraUnion& camera, Variables& vars, Scene& scene,
	Vector3(Renderer::* miss)(const Ray&, Scene&, Variables&, int),
	Vector3(Renderer::* illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&))
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
	for (Shape& shape : scene.shapes)
	{
		src_shapes.push_back(std::make_shared<Shape>(shape));
	}
	pcg32_state rng_bvh = init_pcg32();

	Timer build_time;
	tick(build_time);


	if (m_Vars.parallel_counts_bvh > 1 && src_shapes.size() > 512)
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
					Vector2 offset = Vector2(0.5, 0.5);
					Ray ray = std::visit([=](auto& cam) {return cam.CalculateRayDirections(x, y, offset, rng); }, camera);

					color = TraceRay_BVH_Path(ray, scene, true, vars.max_depth, miss, illumination, rng);

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

						color += TraceRay_BVH_Path(ray, scene, true, vars.max_depth, miss, illumination, rng);
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
						color += TraceRay_BVH_Path(ray, scene, true, vars.max_depth, miss, illumination, rng);

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
							color += TraceRay_BVH_Path(ray, scene, true, vars.max_depth, miss, illumination, rng);
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

void Renderer::Render_BVH_Path_One_Sample(CameraUnion& camera, Variables& vars, Scene& scene,
	Vector3(Renderer::* miss)(const Ray&, Scene&, Variables&, int),
	Vector3(Renderer::* illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&))
{
	m_Vars = vars;
	if (m_Sampler == nullptr)
		m_Sampler = new One_Mix_Sampler();

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
	for (Shape& shape : scene.shapes)
	{
		src_shapes.push_back(std::make_shared<Shape>(shape));
	}
	pcg32_state rng_bvh = init_pcg32();

	Timer build_time;
	tick(build_time);


	if (m_Vars.parallel_counts_bvh > 1 && src_shapes.size() > 512)
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
					Vector2 offset = Vector2(0.5, 0.5);
					Ray ray = std::visit([=](auto& cam) {return cam.CalculateRayDirections(x, y, offset, rng); }, camera);
					color = TraceRay_BVH_Path_One_Sample(ray, scene, true, vars.max_depth, miss, illumination, rng);

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

						color += TraceRay_BVH_Path_One_Sample(ray, scene, true, vars.max_depth, miss, illumination, rng);
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
						color += TraceRay_BVH_Path_One_Sample(ray, scene, true, vars.max_depth, miss, illumination, rng);

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
							color += TraceRay_BVH_Path_One_Sample(ray, scene, true, vars.max_depth, miss, illumination, rng);
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
	Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng)
{
	if (!FindNearstIntersection(ray, scene, m_Vars.t_min, m_Vars.t_max, rng)) // Miss
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
	Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng,
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
	Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng)
{
	if (!FindNearstIntersection_BVH(ray, scene, m_BVH, m_Vars.t_min, m_Vars.t_max, rng)) // Miss
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

Vector3 Renderer::TraceRay_BVH_Path(Ray& ray, Scene& scene, bool isReflect, int max_depth,
	Vector3(Renderer::* miss)(const Ray&, Scene&, Variables&, int),
	Vector3(Renderer::* illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng)
{
	if (!FindNearstIntersection_BVH(ray, scene, m_BVH, m_Vars.t_min, m_Vars.t_max, rng)) // Miss
	{
		return (this->*miss)(ray, scene, m_Vars, max_depth);
	}
	else
	{
		if (isReflect)
			return HitNearst_BVH_Path(ray, scene, true, max_depth, miss, illumination, rng);
		else
			return HitNearst_BVH_Path(ray, scene, false, max_depth, miss, illumination, rng);
	}
}

Vector3 Renderer::TraceRay_BVH_Path_One_Sample(Ray& ray, Scene& scene, bool isReflect, int max_depth,
	Vector3(Renderer::* miss)(const Ray&, Scene&, Variables&, int),
	Vector3(Renderer::* illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng)
{
	if (!FindNearstIntersection_BVH(ray, scene, m_BVH, m_Vars.t_min, m_Vars.t_max, rng)) // Miss
	{
		return (this->*miss)(ray, scene, m_Vars, max_depth);
	}
	else
	{
		if (isReflect)
			return HitNearst_BVH_Path_One_Sample(ray, scene, true, max_depth, miss, illumination, rng);
		else
			return HitNearst_BVH_Path_One_Sample(ray, scene, false, max_depth, miss, illumination, rng);
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
	Vector3 (Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng)
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

	Vector3 color = (this->*illumination)(ray, isReflect, hit_point, normal_transformed, NearstObj, scene, m_Vars, rng);

	ray.Origin = hit_point;
	ray.object.reset();
	ray.distance = 0.;
	if (isReflect)
	{
		Material material = scene.materials[material_id];
		Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
		Vector3 surface_color;
		if (m_color.index() == 0) // RGB color
			surface_color = std::get<Vector3>(m_color);
		else // Texture
		{
			Texture& texture = std::get<Texture>(m_color);
			surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
		}
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
		{
			ray.Direction = reflect(ray.Direction, normal_transformed);
			// Fuzzed
			//ray.Direction += 0.1*sample_in_unit_sphere;
			//ray.Direction = normalize(ray.Direction);
		}

		else if (material.index() == 6) // glass
		{
			Glass& material_glass = std::get<Glass>(material);
			Real index_of_refraction = material_glass.eta;

			if (!isFrontFace(ray, normal_transformed))
			{
				index_of_refraction = 1. / material_glass.eta;
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
		else if (material.index() == 7) // volume
		{
			ray.Direction = sample_in_unit_sphere;
			ray.Direction = normalize(ray.Direction);
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
	Vector3(Renderer::*illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng)
{
	Shape& NearstObj = *ray.object;
	Vector3 hit_point_original = HitPointOriginal(ray, NearstObj);
	Vector3 hit_point = ray.Origin + ray.distance * ray.Direction;


	int material_id;
	Vector3 normal_transformed;
	int area_light_id = std::visit([&](auto& shape) {return shape.area_light_id; }, NearstObj);
	if (area_light_id != -1) // is area_light
	{
		Vector3 radiance = std::get<ParsedDiffuseAreaLight>(scene.lights[area_light_id]).radiance;
		return radiance;
	}
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
		if(m_Vars.shading_normal) // has cached normal
		{
			Vector4 normal_transformed_4 = GetNormalByHitPoint_Smooth(hit_point_original, triangle);
			normal_transformed = Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z);
		}
		else
		{
			Vector4 normal_transformed_4 = GetNormalByHitPoint(hit_point_original, triangle);
			normal_transformed = Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z);
		}

		material_id = triangle.mesh->material_id;
	}

	//ray.Origin = hit_point + normal_transformed * 0.001;

	Vector3 color = (this->*illumination)(ray, isReflect, hit_point, normal_transformed, NearstObj, scene, m_Vars, rng);

	ray.Origin = hit_point;
	ray.object.reset();
	ray.distance = 0.;
	if (isReflect)
	{
		//if (NearstObj.index() == 0) // sphere
		//	material_id = std::get<ParsedSphere>(NearstObj).material_id;
		//else // triangle
		//	material_id = std::get<Triangle>(NearstObj).mesh->material_id;
		if (dot(ray.Direction, normal_transformed) > 0)
		{
			normal_transformed = -normal_transformed;
		}
		Material material = scene.materials[material_id];
		Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
		Vector3 surface_color;
		if (m_color.index() == 0) // RGB color
			surface_color = std::get<Vector3>(m_color);
		else // Texture
		{
			Texture& texture = std::get<Texture>(m_color);
			surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
		}
		Real noize_theta = next_pcg32_real<Real>(rng) * c_PI;
		Real noize_phi = next_pcg32_real<Real>(rng) * 2 * c_PI;
		Vector3 sample_in_unit_sphere{ std::sin(noize_theta) * std::cos(noize_phi),
		std::cos(noize_theta),
		std::sin(noize_theta) * std::sin(noize_phi) };


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
			surface_color = surface_color + (Vector3(1., 1., 1.) - surface_color) * 
				pow(1. - dot(normal_transformed,ray.Direction), 5);

		}
		else if (material.index() == 2) // plastic
		{
			ray.Direction = reflect(ray.Direction, normal_transformed);
			Plastic& material_plastic = std::get<Plastic>(material);
			Real index_of_refraction = material_plastic.eta;
			Real cos_theta = max(dot(ray.Direction, normal_transformed), 0.);
			//std::cout << "cos = " << cos_theta << std::endl;
			Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
			Real F = F_0 + (1 - F_0) * pow(1 - cos_theta, 5);
			surface_color = Vector3(F, F, F);
			//surface_color = surface_color + (1 - F_0) * pow(1 - cos_theta, 5);

		}
		else if (material.index() == 6) // glass
		{
			Glass& material_glass = std::get<Glass>(material);
			Real index_of_refraction = material_glass.eta;

			if (!isFrontFace(ray, normal_transformed))
			{
				index_of_refraction = 1. / material_glass.eta;
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
		else if (material.index() == 7)
		{
			// Fuzzed
			ray.Direction = sample_in_unit_sphere;
			ray.Direction = normalize(ray.Direction);

			/*return color;*/
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

Vector3 Renderer::HitNearst_BVH_Path(Ray& ray, Scene& scene, bool isReflect, int max_depth,
	Vector3(Renderer::* miss)(const Ray&, Scene&, Variables&, int),
	Vector3(Renderer::* illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng)
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
		if (triangle.mesh->normals.size() > 0 && m_Vars.shading_normal) // has cached normal
		{
			Vector4 normal_transformed_4 = GetNormalByHitPoint_Smooth(hit_point_original, triangle);
			normal_transformed = Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z);
		}
		else
		{
			Vector4 normal_transformed_4 = GetNormalByHitPoint(hit_point_original, triangle);
			normal_transformed = Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z);
		}

		material_id = triangle.mesh->material_id;
	}

	//ray.Origin = hit_point + normal_transformed * 0.001;

	Vector3 color = (this->*illumination)(ray, isReflect, hit_point, normal_transformed, NearstObj, scene, m_Vars, rng);

	ray.Origin = hit_point;
	ray.object.reset();
	ray.distance = 0.;
	if (isReflect)
	{
		//if (NearstObj.index() == 0) // sphere
		//	material_id = std::get<ParsedSphere>(NearstObj).material_id;
		//else // triangle
		//	material_id = std::get<Triangle>(NearstObj).mesh->material_id;
		if (dot(ray.Direction, normal_transformed) > 0)
		{
			normal_transformed = -normal_transformed;
		}
		Material material = scene.materials[material_id];
		Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
		Vector3 surface_color;
		if (m_color.index() == 0) // RGB color
			surface_color = std::get<Vector3>(m_color);
		else // Texture
		{
			Texture& texture = std::get<Texture>(m_color);
			surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
		}
		Real noize_theta = next_pcg32_real<Real>(rng) * c_PI;
		Real noize_phi = next_pcg32_real<Real>(rng) * 2 * c_PI;
		Vector3 sample_in_unit_sphere{ std::sin(noize_theta) * std::cos(noize_phi),
		std::cos(noize_theta),
		std::sin(noize_theta) * std::sin(noize_phi) };


		if (material.index() == 0) // diffuse
		{
			if (0 == max_depth)
				return color;
			int sqrt_N = sqrt(m_Vars.brdf_samples);
			Real u1;
			Real u2;
			for (int i = 0; i < sqrt_N; i++) {
				for (int j = 0; j < sqrt_N; j++) {
					Real u1_ori = next_pcg32_real<Real>(rng);
					Real u2_ori = next_pcg32_real<Real>(rng);
					u1 = (i + u1_ori) / sqrt_N;
					u2 = (j + u2_ori) / sqrt_N;

					Real theta = acos(1. - 2. * u1)/2.;	
					Real phi = 2. * c_PI * u2;
					Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
					Vector3 z_axis = normal_transformed;
					Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
					Vector3 y_axis = cross(z_axis, x_axis);
					Vector3 sample_direction = sample_direction_ori.x * x_axis +
												sample_direction_ori.y * y_axis +
												sample_direction_ori.z * z_axis;
					sample_direction = normalize(sample_direction);
					Real cos_light = max(dot(sample_direction, normal_transformed), 0.);
					Real pdf_inv = c_PI / (Real) m_Vars.brdf_samples;
					ray.Direction = sample_direction;

					color += surface_color * c_INVPI * pdf_inv * TraceRay_BVH_Path(ray, scene, true, max_depth-1, miss, illumination, rng);
				}
				
			}
			max_depth -= 1;
			return color;
		}
		else if (material.index() == 1) // mirror
		{
			if (0 == max_depth)
				return color;
			ray.Direction = reflect(ray.Direction, normal_transformed);
			// Fuzzed
			//ray.Direction += 0.1*sample_in_unit_sphere;
			//ray.Direction = normalize(ray.Direction);
			surface_color = surface_color + (Vector3(1., 1., 1.) - surface_color) *
				pow(1. - dot(normal_transformed, ray.Direction), 5);
			color += surface_color * TraceRay_BVH_Path(ray, scene, true, max_depth - 1, miss, illumination, rng);

			max_depth -= 1;
			return color;
		}
		else if (material.index() == 2) // plastic
		{
			if (0 == max_depth)
				return color;
			Plastic& material_plastic = std::get<Plastic>(material);
			Real index_of_refraction = material_plastic.eta;
			Real cos_theta = max(dot(reflect(ray.Direction, normal_transformed), normal_transformed), 0.);
			//std::cout << "cos = " << cos_theta << std::endl;
			Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
			Real F = F_0 + (1. - F_0) * pow(1. - cos_theta, 5);
			//std::cout << "F = " << F << std::endl;

			Real random_f = next_pcg32_real<Real>(rng);
			if(random_f <= F) //specular
			{
				//std::cout << "F = " << F << ", random = " << random_f << std::endl;
				ray.Direction = reflect(ray.Direction, normal_transformed);
				// Fuzzed
				//ray.Direction += 0.1*sample_in_unit_sphere;
				//ray.Direction = normalize(ray.Direction);
				surface_color = Vector3(F, F, F);
				color += (1. / F) * surface_color * TraceRay_BVH_Path(ray, scene, true, max_depth - 1, miss, illumination, rng);
			}
			else //diffuse
			{
				int sqrt_N = sqrt(m_Vars.brdf_samples);
				Real u1;
				Real u2;
				for (int i = 0; i < sqrt_N; i++) {
					for (int j = 0; j < sqrt_N; j++) {
						Real u1_ori = next_pcg32_real<Real>(rng);
						Real u2_ori = next_pcg32_real<Real>(rng);
						u1 = (i + u1_ori) / sqrt_N;
						u2 = (j + u2_ori) / sqrt_N;

						Real theta = acos(1. - 2. * u1) / 2.;
						Real phi = 2. * c_PI * u2;
						Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
						Vector3 z_axis = normal_transformed;
						Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
						Vector3 y_axis = cross(z_axis, x_axis);
						Vector3 sample_direction = sample_direction_ori.x * x_axis +
							sample_direction_ori.y * y_axis +
							sample_direction_ori.z * z_axis;
						sample_direction = normalize(sample_direction);
						Real cos_light = max(dot(sample_direction, normal_transformed), 0.);
						Real pdf_inv = c_PI / (Real)m_Vars.brdf_samples;
						ray.Direction = sample_direction;

						color += surface_color * c_INVPI * pdf_inv * TraceRay_BVH_Path(ray, scene, true, max_depth - 1, miss, illumination, rng);
					}
				}
			}
			max_depth -= 1;
			return color;
		}
		else if (material.index() == 3) // phong
		{
			Phong& material_phong = std::get<Phong>(material);
			if (0 == max_depth)
				return color;

			int sqrt_N = sqrt(m_Vars.brdf_samples);
			Real u1;
			Real u2;
			for (int i = 0; i < sqrt_N; i++) {
				for (int j = 0; j < sqrt_N; j++) {
					Real u1_ori = next_pcg32_real<Real>(rng);
					Real u2_ori = next_pcg32_real<Real>(rng);
					u1 = (i + u1_ori) / sqrt_N;
					u2 = (j + u2_ori) / sqrt_N;

					Real theta = acos(pow(1. - u1, 1. / (material_phong.exponent + 1)));
					Real phi = 2. * c_PI * u2;
					Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
					Vector3 z_axis = reflect(ray.Direction, normal_transformed);
					Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
					Vector3 y_axis = cross(z_axis, x_axis);
					Vector3 sample_direction = sample_direction_ori.x * x_axis +
						sample_direction_ori.y * y_axis +
						sample_direction_ori.z * z_axis;
					sample_direction = normalize(sample_direction);
					if (dot(sample_direction, normal_transformed) < 0)
						continue;
					Real cos_light = max(dot(sample_direction, normal_transformed), 0.);
					Real pdf_cos_hemisphere = 2 * c_PI * (1.  / (material_phong.exponent + 1.)) / (Real)m_Vars.brdf_samples;
					ray.Direction = sample_direction;

					color += surface_color * (material_phong.exponent + 1.) * (c_INVPI / 2.) * pdf_cos_hemisphere * TraceRay_BVH_Path(ray, scene, true, max_depth - 1, miss, illumination, rng);
				}
			}
			max_depth -= 1;
			return color;
		}
		else if (material.index() == 4) // blingphong
		{
			BlinnPhong& material_blinnphong = std::get<BlinnPhong>(material);
			if (0 == max_depth)
				return color;

			int sqrt_N = sqrt(m_Vars.brdf_samples);
			Real u1;
			Real u2;
			for (int i = 0; i < sqrt_N; i++) {
				for (int j = 0; j < sqrt_N; j++) {
					Real u1_ori = next_pcg32_real<Real>(rng);
					Real u2_ori = next_pcg32_real<Real>(rng);
					u1 = (i + u1_ori) / sqrt_N;
					u2 = (j + u2_ori) / sqrt_N;

					Real alpha = material_blinnphong.exponent;
					Real theta = acos(pow(1. - u1, 1. / (alpha + 1)));
					Real phi = 2. * c_PI * u2;
					Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
					Vector3 z_axis = normal_transformed;
					Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
					Vector3 y_axis = cross(z_axis, x_axis);
					Vector3 sample_direction = sample_direction_ori.x * x_axis +
						sample_direction_ori.y * y_axis +
						sample_direction_ori.z * z_axis;
					sample_direction = normalize(sample_direction);
					Vector3 half_vector = sample_direction;
					Vector3 w_o = reflect(ray.Direction, half_vector);
					Real cos_norm_half = dot(half_vector, normal_transformed);
					Real cos_wo_half = dot(w_o, half_vector);

					surface_color = surface_color + pow(1. - cos_wo_half, 5.) * (Vector3(1., 1., 1.) - surface_color);

					if (dot(w_o, normal_transformed) < 0)
						continue;

					Real pdf_inv = 8 * c_PI * cos_wo_half * 
						(1. / ((alpha + 1.) * pow(cos_norm_half, alpha))) / (Real)m_Vars.brdf_samples;
					ray.Direction = w_o;

					color += surface_color * (alpha + 2.) * (c_INVPI / (4. * 2.-pow(2., -alpha/2.))) * pow(cos_norm_half, alpha) *
						pdf_inv * TraceRay_BVH_Path(ray, scene, true, max_depth - 1, miss, illumination, rng);
				}
			}
			max_depth -= 1;
			return color;
		}
		else if (material.index() == 5) // blingphongmicro
		{
			
			BlinnPhongMicrofacet& material_blinnphongMicro = std::get<BlinnPhongMicrofacet>(material);
			if (0 == max_depth)
				return color;

			int sqrt_N = sqrt(m_Vars.brdf_samples);
			Real u1;
			Real u2;
			for (int i = 0; i < sqrt_N; i++) {
				for (int j = 0; j < sqrt_N; j++) {
					Real u1_ori = next_pcg32_real<Real>(rng);
					Real u2_ori = next_pcg32_real<Real>(rng);
					u1 = (i + u1_ori) / sqrt_N;
					u2 = (j + u2_ori) / sqrt_N;

					Real alpha = material_blinnphongMicro.exponent;
					Real theta = acos(pow(1. - u1, 1. / (alpha + 1)));
					Real phi = 2. * c_PI * u2;
					Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
					Vector3 z_axis = normal_transformed;
					Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
					Vector3 y_axis = cross(z_axis, x_axis);
					Vector3 sample_direction = sample_direction_ori.x * x_axis +
						sample_direction_ori.y * y_axis +
						sample_direction_ori.z * z_axis;
					sample_direction = normalize(sample_direction);

					Vector3 half_vector = sample_direction;
					Vector3 w_o = reflect(ray.Direction, half_vector);
					Real cos_norm_half = dot(half_vector, normal_transformed);
					Real cos_wo_half = dot(w_o, half_vector);
					Real cos_wi_half = dot(-ray.Direction, half_vector);
					Real cos_wi_normal = dot(-ray.Direction, normal_transformed);
					Real theta_i_normal = unit_angle(-ray.Direction, normal_transformed);
					Real theta_o_normal = unit_angle(w_o, normal_transformed);
					Real G_i = cos_wi_half > 0 ? GetGeoConfig(theta_i_normal, alpha) : 0.;
					Real G_o = cos_wo_half > 0 ? GetGeoConfig(theta_o_normal, alpha) : 0.;
					Real G_w = G_i * G_o;
					surface_color = surface_color + pow(1. - cos_wo_half, 5.) * (Vector3(1., 1., 1.) - surface_color);

					if (dot(w_o, normal_transformed) < 0)
						continue;

					Real pdf_inv = 8 * c_PI * cos_wo_half * 
						(1. / ((alpha + 1.) * pow(cos_norm_half, alpha))) / (Real)m_Vars.brdf_samples;

					ray.Direction = w_o;

					color += surface_color * (alpha + 2.) * (c_INVPI/8.) * pow(cos_norm_half, alpha) * G_w * (1. / cos_wi_normal) *
						pdf_inv * TraceRay_BVH_Path(ray, scene, true, max_depth - 1, miss, illumination, rng);
				}
			}
			max_depth -= 1;
			return color;
		}
		else if (material.index() == 6) // glass
		{
			Glass& material_glass = std::get<Glass>(material);
			Real index_of_refraction = material_glass.eta;

			if (!isFrontFace(ray, normal_transformed))
			{
				index_of_refraction = 1. / material_glass.eta;
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
		else if (material.index() == 7)
		{
			// Fuzzed
			ray.Direction = sample_in_unit_sphere;
			ray.Direction = normalize(ray.Direction);

			/*return color;*/
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

Vector3 Renderer::HitNearst_BVH_Path_One_Sample(Ray& ray, Scene& scene, bool isReflect, int max_depth,
	Vector3(Renderer::* miss)(const Ray&, Scene&, Variables&, int),
	Vector3(Renderer::* illumination)(Ray&, bool, Vector3, Vector3, Shape&, Scene&, Variables&, pcg32_state&), pcg32_state& rng)
{
	Shape& NearstObj = *ray.object;
	Vector3 hit_point_original = HitPointOriginal(ray, NearstObj);
	Vector3 hit_point = ray.Origin + ray.distance * ray.Direction;
	ray.radius += ray.spread * ray.distance;

	int material_id;
	Vector3 normal_transformed;
	Vector3 normal_shading;
	if (NearstObj.index() == 0) // is sphere
	{
		ParsedSphere& sphere = std::get<ParsedSphere>(NearstObj);
		material_id = sphere.material_id;
		Vector4 normal_original = GetNormalByHitPoint(hit_point_original, sphere);
		Vector4 normal_transformed_4 = Vector4(transpose(inverse(
			translate(Vector3(sphere.position.x, sphere.position.y, sphere.position.z)))) * normal_original);
		normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));

		Vector4 normal_shading_ori = GetNormalByHitPoint(hit_point_original, sphere, scene.materials[material_id], ray);
		Vector4 normal_shading_4 = Vector4(transpose(inverse(
			translate(Vector3(sphere.position.x, sphere.position.y, sphere.position.z)))) * normal_shading_ori);
		normal_shading = normalize(Vector3(normal_shading_4.x, normal_shading_4.y, normal_shading_4.z));

	}
	else //is triangle
	{
		Triangle& triangle = std::get<Triangle>(NearstObj);
		material_id = triangle.mesh->material_id;
		if (triangle.mesh->normals.size() > 0 && m_Vars.shading_normal) // has cached normal
		{
			Vector4 normal_transformed_4 = GetNormalByHitPoint_Smooth(hit_point_original, triangle);
			normal_transformed = Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z);

			Vector4 normal_shading_4 = GetNormalByHitPoint_Smooth(hit_point_original, triangle, scene.materials[material_id], ray);
			normal_shading = normalize(Vector3(normal_shading_4.x, normal_shading_4.y, normal_shading_4.z));
		}
		else
		{
			Vector4 normal_transformed_4 = GetNormalByHitPoint(hit_point_original, triangle);
			normal_transformed = Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z);

			Vector4 normal_shading_4 = GetNormalByHitPoint(hit_point_original, triangle, scene.materials[material_id], ray);
			normal_shading = normalize(Vector3(normal_shading_4.x, normal_shading_4.y, normal_shading_4.z));
		}

		
	}

	Material material = scene.materials[material_id];
	//ray.Origin = hit_point + normal_transformed * 0.001;

	Vector3 emit_color = (this->*illumination)(ray, isReflect, hit_point, normal_transformed, NearstObj, scene, m_Vars, rng);

	ray.Origin = hit_point;
	ray.object.reset();
	ray.distance = 0.;

	int area_light_id = std::visit([&](auto& shape) {return shape.area_light_id; }, NearstObj);


	if (isReflect)
	{
		if (dot(ray.Direction, normal_transformed) > 0)
		{
			normal_transformed = -normal_transformed;
			ray.front_face = false;
		}
		else
			ray.front_face = true;
		if (0 == max_depth)
			return emit_color;

		//Material material, Scene& scene, Renderer& renderer, Ray ray, Vector3 normal, Variables& m_Vars
		//m_Sampler = new BRDF_Sampler();
		//if (material.index() == 2)
		//	//std::cout << "Plastic!" << std::endl;
		if(m_Vars.multi_sample) //multi-sample
		{
			Ray ray_copy = ray;
			Vector3 total_color(0., 0., 0.);
			bool is_reflect_brdf, is_reflect_light = false;
			bool is_refract_brdf, is_refract_light = false;

			BRDF_Sampler brdf_sampler;
			Vector3 scatter_direction_brdf = brdf_sampler.SampleDirection(material, ray, normal_transformed, m_Vars, scene, m_BVH, rng, &is_reflect_brdf, &is_refract_brdf);
			Real pdf_brdf = brdf_sampler.GetPDF(material, ray, normal_transformed, m_Vars, scatter_direction_brdf, scene, m_BVH, rng, is_reflect_brdf, is_refract_brdf);
			Vector3 brdf_brdf = brdf_sampler.GetBRDF(material, ray, normal_transformed, m_Vars, scatter_direction_brdf, scene, m_BVH, rng, is_reflect_brdf, is_refract_brdf);


			Light_Sampler light_sampler;
			Vector3 scatter_direction_light = light_sampler.SampleDirection(material, ray_copy, normal_transformed, m_Vars, scene, m_BVH, rng, &is_reflect_light, &is_refract_light);
			Real pdf_light = light_sampler.GetPDF(material, ray_copy, normal_transformed, m_Vars, scatter_direction_light, scene, m_BVH, rng, is_reflect_light, is_refract_light);
			Vector3 brdf_light = light_sampler.GetBRDF(material, ray_copy, normal_transformed, m_Vars, scatter_direction_light, scene, m_BVH, rng, is_reflect_light, is_refract_light);

			Real weight_for_brdf = pdf_brdf / (pdf_brdf + light_sampler.GetPDF(material, ray, normal_transformed, m_Vars, scatter_direction_brdf, scene, m_BVH, rng, is_reflect_brdf, is_refract_brdf));
			Real weight_for_light = pdf_light / (pdf_light + brdf_sampler.GetPDF(material, ray_copy, normal_transformed, m_Vars, scatter_direction_light, scene, m_BVH, rng, is_reflect_light, is_refract_light));

			ray.Direction = scatter_direction_brdf;
			ray_copy.Direction = scatter_direction_light;

			total_color += emit_color;

			if (pdf_brdf > 0 && length(brdf_brdf) > 0)
			{
				total_color += weight_for_brdf * (brdf_brdf * TraceRay_BVH_Path_One_Sample(ray, scene, true, max_depth - 1, miss, illumination, rng) /
					pdf_brdf);
			}

			if (pdf_light > 0 && length(brdf_light) > 0)
			{
				total_color += weight_for_light * (brdf_light * TraceRay_BVH_Path_One_Sample(ray_copy, scene, true, max_depth - 1, miss, illumination, rng) /
					pdf_light);
			}


			return total_color;
		}
		else // one-sample
		{
			bool is_reflect = false;
			bool is_refract = false;
			if (area_light_id != -1)
				return emit_color;
			Vector3 scatter_direction = m_Sampler->SampleDirection(material, ray, normal_transformed, m_Vars, scene, m_BVH, rng, &is_reflect, &is_refract);
			//std::cout << "scatter:(" << scatter_direction.x << "," << scatter_direction.y << "," << scatter_direction.z << ")" << std::endl;
			Real pdf = m_Sampler->GetPDF(material, ray, normal_transformed, m_Vars, scatter_direction, scene, m_BVH, rng, is_reflect, is_refract);
			//std::cout << pdf << std::endl;
			Vector3 brdf = m_Sampler->GetBRDF(material, ray, normal_shading, m_Vars, scatter_direction, scene, m_BVH, rng, is_reflect, is_refract);
			ray.Direction = scatter_direction;

			if (pdf > 0 && length(brdf) > 0)
			{
				return  emit_color + brdf * TraceRay_BVH_Path_One_Sample(ray, scene, true, max_depth - 1, miss, illumination, rng) /
					pdf;
			}
			else
				return emit_color;
		}



		//MIS_Infor Light_infor = Light_Sampling(ray_copy, isReflect, hit_point, normal_transformed, NearstObj, scene, m_Vars, rng);
		//MIS_Infor BRDF_infor = BRDF_Sampling(ray, isReflect, hit_point, normal_transformed, NearstObj, scene, m_Vars, rng);
		//Real p_BRDF = BRDF_infor.PDF_value;
		//Real p_Light = Light_infor.PDF_value;
		//Real proportion = p_BRDF / (p_BRDF + p_Light);

		//std::cout << "p_BRDF:" << p_BRDF << ", p_Light:" << p_Light << std::endl;
		//proportion = 0.5;
		//if(random_p < 0.5) // BRDF sampling
		//{

		//	Vector3 T_value_next = BRDF_infor.T_value_next;
		//	return  emit_color + 2. * proportion * T_value_next * TraceRay_BVH_Path_One_Sample(ray, scene, true, max_depth - 1, miss, illumination, rng);
		//	//return  emit_color + 2.* T_value_next * TraceRay_BVH_Path_One_Sample(ray, scene, true, max_depth - 1, miss, illumination, rng);

		//}
		//else // Light sampling
		//{
		//	Vector3 L_value = Light_infor.L_value;
		//	Vector3 T_value_next = Light_infor.T_value_next;
		//	//if(length(ray_copy.Direction) == 0.) // stop recursion
		//		//return  emit_color + 2.* (1. - proportion) * L_value;
		//		return  emit_color + 2. * (1. - proportion) * (T_value_next * TraceRay_BVH_Path_One_Sample(ray_copy, scene, true, max_depth - 1, miss, illumination, rng));

		//	//else
		//	//	return emit_color + 2. * (1. - proportion) * (L_value +
		//	//		T_value_next * TraceRay_BVH_Path_One_Sample(ray_copy, scene, true, max_depth - 1, miss, illumination, rng));
		//		//return emit_color + 2.* (L_value +
		//		//	T_value_next * TraceRay_BVH_Path_One_Sample(ray_copy, scene, true, max_depth - 1, miss, illumination, rng));
		//}

	}
}

Vector3 Renderer::Miss(const Ray& ray, Scene& scene, int max_depth)
{
	return Vector3(0., 0., 0.);
	//return Vector4(0.4f, 0.9f, 1.0f, 0.9f);
	//return Vector4(0.2f, 0.45f, 0.5f, 0.9f);
}


bool Renderer::FindNearstIntersection(Ray& ray, Scene& scene, Real t_min, Real t_max, pcg32_state& rng)
{
	for (Shape& shape : scene.shapes)
	{
		if (ray.FindIntersection(shape, t_min, t_max, scene, rng))
		{
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


bool Renderer::FindNearstIntersection_BVH(Ray& ray, Scene& scene, BVH& node, Real t_min, Real t_max, pcg32_state& rng)
{
	if (node.isHit(ray, t_min, t_max, scene, rng))
	{
		if (ray.object)
			return true;
	}
	
	return false;
}

MIS_Infor Renderer::Light_Sampling(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}


	Vector3 color{ 0., 0., 0. };
	//parallel_for([&](int counts)
	//	{
	//	}, (int64_t)10);
	Vector3 ray_direction_ori = ray.Direction;
	std::vector<Vector3> color_weight;
	std::vector<Real> PDF_value;
	for (ParsedLight light : scene.lights)
	{
		std::vector<Real> d;
		std::vector<Real> visibility;
		std::vector<Real> pdf_and_cos;
		std::vector<Real> pdf_inv;
		std::vector<Vector3> position;
		std::vector<Vector3> intensity;
		Vector3 light_direction = Vector3(0., 0., 0.);
		if (light.index() == 0) // is PointLight
		{
			ParsedPointLight point_light = std::get<ParsedPointLight>(light);
			position.push_back(point_light.position);
			intensity.push_back(point_light.intensity);
			pdf_and_cos.push_back(1.);
			pdf_inv.push_back(1.);
			light_direction = HitPoint - point_light.position;
			d.push_back(length(light_direction));
			Ray shadow_ray = { HitPoint, -normalize(light_direction) };
			if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
				visibility.push_back(1.);
			else
				visibility.push_back(0);
		}
		else // is area light
		{
			ParsedDiffuseAreaLight area_light = std::get<ParsedDiffuseAreaLight>(light);
			if (scene.src_shapes[area_light.shape_id]->index() == 0) // sphere
			{
				int sqrt_N = sqrt(vars.area_light_samples);
				Real u1;
				Real u2;
				for (int i = 0; i < sqrt_N; i++) {
					for (int j = 0; j < sqrt_N; j++) {
						Real u1_ori = next_pcg32_real<Real>(rng);
						Real u2_ori = next_pcg32_real<Real>(rng);
						u1 = (i + u1_ori) / sqrt_N;
						u2 = (j + u2_ori) / sqrt_N;

						ParsedSphere& light_sphere = std::get<ParsedSphere>(*scene.src_shapes[area_light.shape_id]);
						Real dc = length(light_sphere.position - HitPoint);
						Real cos_theta_max = sqrt(max(0., (light_sphere.radius * light_sphere.radius) / dc * dc));

						Real theta = acos(1. + (cos_theta_max - 1.) * u1);
						//std::cout << "theta:" << theta << std::endl;
						//Real theta = acos(1. - 2. * u1);
						Real phi = 2. * c_PI * u2;
						Real r = light_sphere.radius;
						Real cos_theta = cos(theta);
						Real sin_theta = sin(theta);
						Vector3 offset = { sin_theta * cos(phi), sin_theta * sin(phi), cos_theta };
						Vector3 z_axis = normalize(HitPoint - light_sphere.position);
						Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
						Vector3 y_axis = cross(z_axis, x_axis);
						Vector3 sample_direction = offset.x * x_axis +
							offset.y * y_axis +
							offset.z * z_axis;
						sample_direction = normalize(sample_direction);
						offset = sample_direction * light_sphere.radius;
						Vector3 hit_p = light_sphere.position + offset;
						position.push_back(hit_p);
						light_direction = HitPoint - hit_p;
						d.push_back(length(light_direction));
						intensity.push_back(area_light.radiance);
						Ray shadow_ray = { HitPoint, -normalize(light_direction) };
						if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
							visibility.push_back(1.);
						else
						{
							int shadow_hit_area_light_id = std::visit([&](auto shape) {return shape.area_light_id; }, *shadow_ray.object);
							if (shadow_hit_area_light_id == light_sphere.area_light_id)
								visibility.push_back(1.);
							else
								visibility.push_back(0);
						}

						light_direction = normalize(light_direction);
						Vector4 normal_original = GetNormalByHitPoint(offset, light_sphere);
						Vector4 normal_transformed_4 = Vector4(transpose(inverse(
							translate(normalize(Vector3(light_sphere.position.x, light_sphere.position.y, light_sphere.position.z))))) * normal_original);
						Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
						Real cos_light = max(dot(light_direction, normal_transformed), 0.);
						pdf_and_cos.push_back(cos_light * 2. * (1. - cos_theta_max) * c_PI * light_sphere.radius * light_sphere.radius / (Real)vars.area_light_samples);
						pdf_inv.push_back(2. * (1. - cos_theta_max) * c_PI * light_sphere.radius * light_sphere.radius / (Real)vars.area_light_samples);

					}
				}
			}
			else // trianglemesh
			{
				ParsedTriangleMesh& light_mesh = std::get<ParsedTriangleMesh>(*scene.src_shapes[area_light.shape_id]);
				for (int i = 0; i < light_mesh.indices.size(); i++)
				{
					if (light_mesh.indices.size() > 100)
					{
						if (i % light_mesh.indices.size() != 0)
							continue;
					}
					int sqrt_N = sqrt(vars.area_light_samples);
					Real u1;
					Real u2;
					for (int k = 0; k < sqrt_N; k++) {
						for (int j = 0; j < sqrt_N; j++) {
							Real u1_ori = next_pcg32_real<Real>(rng);
							Real u2_ori = next_pcg32_real<Real>(rng);
							u1 = (k + u1_ori) / sqrt_N;
							u2 = (j + u2_ori) / sqrt_N;
							//std::cout << "u1:" << u1 << ",u2:" << u2 << std::endl;
							Triangle light_triangle = { &light_mesh, i, light_mesh.area_light_id };
							Real b1 = 1. - sqrt(u1);
							Real b2 = u2 * sqrt(u1);
							//std::cout << "b1:" << b1 << ",b2:" << b2 << std::endl;
							Vector3 p1 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][0]];
							Vector3 p2 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][1]];
							Vector3 p3 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][2]];
							Vector3 hit_p = (1. - b1 - b2) * p1 + b1 * p2 + b2 * p3;
							position.push_back(hit_p);
							light_direction = HitPoint - hit_p;
							d.push_back(length(light_direction));
							intensity.push_back(area_light.radiance);
							Ray shadow_ray = { HitPoint, -normalize(light_direction) };
							if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
								visibility.push_back(1.);
							else
							{
								int shadow_hit_area_light_id = std::visit([&](auto shape) {return shape.area_light_id; }, *shadow_ray.object);
								if (shadow_hit_area_light_id == light_mesh.area_light_id)
									visibility.push_back(1.);
								else
									visibility.push_back(0);
							}

							light_direction = normalize(light_direction);
							Vector4 normal_original = GetNormalByHitPoint(hit_p, light_triangle);
							Vector4 normal_transformed_4 = normal_original;
							Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
							Real cos_light = max(dot(light_direction, normal_transformed), 0.);
							pdf_and_cos.push_back(cos_light * GetAreaByShape(light_triangle) / (Real)vars.area_light_samples);
							pdf_inv.push_back(GetAreaByShape(light_triangle) / (Real)vars.area_light_samples);

						}
					}
				}
			}
		}


		//For rainbow color
		//light->light_color = vec4(sin((ray.Origin.x + 10.) / (12.7 / 1.5)), sin((ray.Origin.z + 50.) / (31.8 / 1.5)), cos((ray.Origin.x + 10.) / (12.7 / 1.5)), 1.);

		Vector3 color_from_light = Vector3(0., 0., 0.);
		Vector3 normal = normalize(Normal);
		Real largest_weight = 0.;
		int largest_index = 0;

		if (dot(ray_direction_ori, normal) > 0)
		{
			normal = -normal;
		}

		for (int i = 0; i < position.size(); i++)
		{
			light_direction = normalize(HitPoint - position[i]);
			//PDF_value += visibility[i]  * max(dot(-light_direction, normal), 0.) *
			//	(vars.attenuation_const + vars.attenuation_linear * d[i] + vars.attenuation_quadratic * d[i] * d[i]) / pdf_and_cos[i];

			//Vector3 halfvec = normalize(-direction - light_direction);
			//Vector3 specular = intensity * surface_color *
			//	pow(max(dot(halfvec, normalize(normal)), 0.), 1000000);

			Vector3 transmission{ 0., 0., 0. };
			//specular = specular / (vars.attenuation_const + vars.attenuation_linear * d + vars.attenuation_quadratic * d * d);
			if (material.index() == 0)// diffuse
			{
				Real weight = visibility[i] * (c_INVPI *
					max(dot(-light_direction, normal), 0.)) * pdf_and_cos[i] /
					((vars.attenuation_const + vars.attenuation_linear * d[i] + vars.attenuation_quadratic * d[i] * d[i]));
				color_from_light += surface_color * weight * intensity[i];
				if(largest_weight  < weight)
				{
					largest_weight = weight;
					largest_index = i;
				}
				ray.Direction = -light_direction;

					
			}
			else if (material.index() == 1) // mirror
				ray.Direction = reflect(-ray_direction_ori, normal);
			else if (material.index() == 2) // plastic
			{
				Plastic& material_plastic = std::get<Plastic>(material);
				Real index_of_refraction = material_plastic.eta;
				Real cos_theta = dot(reflect(ray_direction_ori, normal), normal);
				Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
				Real F = F_0 + (1 - F_0) * pow(1 - cos_theta, 5);
				color_from_light += (1. - F) * intensity[i] * visibility[i] * surface_color * (c_INVPI *
					max(dot(-light_direction, normal), 0.)) * pdf_and_cos[i] /
					(vars.attenuation_const + vars.attenuation_linear * d[i] + vars.attenuation_quadratic * d[i] * d[i]);
				ray.Direction = reflect(-ray_direction_ori, normal);
			}
			else if (material.index() == 3) // phong
			{
				ray.Direction = reflect(-ray_direction_ori, normal);
				Phong& material_phong = std::get<Phong>(material);
				if(dot(-light_direction, normal) > 0)
				{
					color_from_light += intensity[i] * visibility[i] * surface_color * ((material_phong.exponent + 1.) * (c_INVPI / 2.) *
						max(pow(dot(-light_direction, ray.Direction), material_phong.exponent), 0.)) * pdf_and_cos[i];
				}

			}
			else if (material.index() == 4) // blingphong
			{
				BlinnPhong& material_blinnphong = std::get<BlinnPhong>(material);
				Real alpha = material_blinnphong.exponent;

				Vector3 w_o = -light_direction;
				Vector3 half_vector = normalize(-ray_direction_ori - light_direction);
				Real cos_norm_half = dot(normal, half_vector);
				Real cos_wo_half = dot(w_o, half_vector);

				Vector3 surface_color_fresnel = surface_color + pow(1. - cos_wo_half, 5.) * (Vector3(1., 1., 1.) - surface_color);
				//color_weight = surface_color_fresnel;
				if (dot(-light_direction, normal) > 0)
				{
					color_from_light += intensity[i] * visibility[i] * surface_color_fresnel *
						((alpha + 2.) * (c_INVPI / (4. * 2. - pow(2., -alpha / 2.))) * pow(cos_norm_half, alpha)) * pdf_and_cos[i] /
						(vars.attenuation_const + vars.attenuation_linear * d[i] + vars.attenuation_quadratic * d[i] * d[i]);
				}
				ray.Direction = w_o;

			}
			else if (material.index() == 5) // blingphongmicro
			{

				BlinnPhongMicrofacet& material_blinnphongMicro = std::get<BlinnPhongMicrofacet>(material);

				Real alpha = material_blinnphongMicro.exponent;
				Vector3 w_o = -light_direction;
				Vector3 half_vector = normalize(-ray.Direction - light_direction);
				Real cos_norm_half = dot(normal, half_vector);
				Real cos_wo_half = dot(w_o, half_vector);

				Vector3 surface_color_fresnel = surface_color + pow(1. - cos_wo_half, 5.) * (Vector3(1., 1., 1.) - surface_color);
				//color_weight = surface_color_fresnel;

				Real cos_wi_half = dot(-ray_direction_ori, half_vector);
				Real cos_wi_normal = dot(-ray_direction_ori, normal);
				Real theta_i_normal = unit_angle(-ray_direction_ori, normal);
				Real theta_o_normal = unit_angle(w_o, normal);
				Real G_i = cos_wi_half > 0 ? GetGeoConfig(theta_i_normal, alpha) : 0.;
				Real G_o = cos_wo_half > 0 ? GetGeoConfig(theta_o_normal, alpha) : 0.;
				Real G_w = G_i * G_o;

				if (dot(-light_direction, Normal) > 0)
				{
					color_from_light += intensity[i] * visibility[i] * surface_color_fresnel *
						((alpha + 2.) * (c_INVPI / 8.) * pow(cos_norm_half, alpha)) * G_w * (1. / cos_wi_normal) * pdf_and_cos[i] /
						(vars.attenuation_const + vars.attenuation_linear * d[i] + vars.attenuation_quadratic * d[i] * d[i]);
				}

				ray.Direction = w_o;
			}
			else if (material.index() == 6) // glass
				color += transmission * visibility[i] * c_INVPI;
			else if (material.index() == 7) // volume
			{
				//color += diffuse * visibility[i] * c_INVPI * max(dot(-light_direction, Vector3(0., 0., 1.)), 0.);
			}
		}

		if (pdf_and_cos[largest_index] > 0 && visibility[largest_index] == 1)
		{
			ray.Direction = -normalize(HitPoint - position[largest_index]);
		}
		if (pdf_and_cos[largest_index] > 0 && dot(-light_direction, normal) > 0)
			PDF_value.push_back(visibility[largest_index] * (vars.attenuation_const + vars.attenuation_linear * d[largest_index] + vars.attenuation_quadratic * d[largest_index] * d[largest_index]) / pdf_and_cos[largest_index]);
		color_weight.push_back(largest_weight * surface_color);

		color += color_from_light;
	}

	Real pdf_v = 0.;
	//color += ambient + emission;
	if (material.index() == 1)
		pdf_v = 0.5;
	else
	{
		for(Real pdf : PDF_value)
		{
			pdf_v += pdf / PDF_value.size();

		}
	}
	Vector3 c_weight = { 0., 0., 0. };

	for(Vector3 weight : color_weight)
	{
		c_weight += weight * (1. / color_weight.size());
	}
	
	return {color, c_weight , pdf_v };
}

MIS_Infor Renderer::BRDF_Sampling(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color_weight
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}
	Vector3 color_weight = { 0., 0., 0. };
	Real noize_theta = next_pcg32_real<Real>(rng) * c_PI;
	Real noize_phi = next_pcg32_real<Real>(rng) * 2 * c_PI;
	Vector3 sample_in_unit_sphere{ std::sin(noize_theta) * std::cos(noize_phi),
	std::cos(noize_theta),
	std::sin(noize_theta) * std::sin(noize_phi) };

	Real PDF_value = 0.;

	if (material.index() == 0) // diffuse
	{
		int sqrt_N = sqrt(m_Vars.brdf_samples);
		Real u1;
		Real u2;
		for (int i = 0; i < sqrt_N; i++) {
			for (int j = 0; j < sqrt_N; j++) {
				Real u1_ori = next_pcg32_real<Real>(rng);
				Real u2_ori = next_pcg32_real<Real>(rng);
				u1 = (i + u1_ori) / sqrt_N;
				u2 = (j + u2_ori) / sqrt_N;

				Real theta = acos(1. - 2. * u1) / 2.;
				Real phi = 2. * c_PI * u2;
				Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
				Vector3 z_axis = Normal;
				Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
				Vector3 y_axis = cross(z_axis, x_axis);
				Vector3 sample_direction = sample_direction_ori.x * x_axis +
					sample_direction_ori.y * y_axis +
					sample_direction_ori.z * z_axis;
				sample_direction = normalize(sample_direction);
				Real cos_wo_normal = max(dot(sample_direction, Normal), 0.);
				Real pdf_inv = c_PI  * (1. / cos_wo_normal)/ (Real)m_Vars.brdf_samples;
				ray.Direction = sample_direction;
				PDF_value = 1. / pdf_inv;
				color_weight += (surface_color * c_INVPI * cos_wo_normal) * pdf_inv;
			}

		}
	}
	else if (material.index() == 1) // mirror
	{
		ray.Direction = reflect(ray.Direction, Normal);
		// Fuzzed
		//ray.Direction += 0.1*sample_in_unit_sphere;
		//ray.Direction = normalize(ray.Direction);
		surface_color = surface_color + (Vector3(1., 1., 1.) - surface_color) *
			pow(1. - dot(Normal, ray.Direction), 5);
		color_weight += surface_color;
		PDF_value = 0.5;
	}
	else if (material.index() == 2) // plastic
	{
		Plastic& material_plastic = std::get<Plastic>(material);
		Real index_of_refraction = material_plastic.eta;
		Real cos_theta = max(dot(reflect(ray.Direction, Normal), Normal), 0.);
		//std::cout << "cos = " << cos_theta << std::endl;
		Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
		Real F = F_0 + (1. - F_0) * pow(1. - cos_theta, 5);
		//std::cout << "F = " << F << std::endl;

		Real random_f = next_pcg32_real<Real>(rng);
		if (random_f <= F) //specular
		{
			//std::cout << "F = " << F << ", random = " << random_f << std::endl;
			ray.Direction = reflect(ray.Direction, Normal);
			// Fuzzed
			//ray.Direction += 0.1*sample_in_unit_sphere;
			//ray.Direction = normalize(ray.Direction);
			surface_color = Vector3(F, F, F);
			color_weight += (1. / F) * surface_color;
		}
		else //diffuse
		{
			int sqrt_N = sqrt(m_Vars.brdf_samples);
			Real u1;
			Real u2;
			for (int i = 0; i < sqrt_N; i++) {
				for (int j = 0; j < sqrt_N; j++) {
					Real u1_ori = next_pcg32_real<Real>(rng);
					Real u2_ori = next_pcg32_real<Real>(rng);
					u1 = (i + u1_ori) / sqrt_N;
					u2 = (j + u2_ori) / sqrt_N;

					Real theta = acos(1. - 2. * u1) / 2.;
					Real phi = 2. * c_PI * u2;
					Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
					Vector3 z_axis = Normal;
					Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
					Vector3 y_axis = cross(z_axis, x_axis);
					Vector3 sample_direction = sample_direction_ori.x * x_axis +
						sample_direction_ori.y * y_axis +
						sample_direction_ori.z * z_axis;
					sample_direction = normalize(sample_direction);
					Real cos_light = max(dot(sample_direction, Normal), 0.);
					Real pdf_inv = c_PI * (1. / cos_light) / (Real)m_Vars.brdf_samples;
					ray.Direction = sample_direction;

					PDF_value = 1. / pdf_inv;

					color_weight += (surface_color * c_INVPI * cos_light) * pdf_inv;
				}
			}
		}
	}
	else if (material.index() == 3) // phong
	{
		Phong& material_phong = std::get<Phong>(material);

		int sqrt_N = sqrt(m_Vars.brdf_samples);
		Real u1;
		Real u2;
		Real alpha = material_phong.exponent;
		for (int i = 0; i < sqrt_N; i++) {
			for (int j = 0; j < sqrt_N; j++) {
				Real u1_ori = next_pcg32_real<Real>(rng);
				Real u2_ori = next_pcg32_real<Real>(rng);
				u1 = (i + u1_ori) / sqrt_N;
				u2 = (j + u2_ori) / sqrt_N;

				Real theta = acos(pow(1. - u1, 1. / (alpha + 1)));
				Real phi = 2. * c_PI * u2;
				Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
				Vector3 z_axis = reflect(ray.Direction, Normal);
				Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
				Vector3 y_axis = cross(z_axis, x_axis);
				Vector3 sample_direction = sample_direction_ori.x * x_axis +
					sample_direction_ori.y * y_axis +
					sample_direction_ori.z * z_axis;
				sample_direction = normalize(sample_direction);
				if (dot(sample_direction, Normal) < 0)
				{
					PDF_value = 0.;
					continue;
				}
					
				Real cos_wo_normal = max(dot(sample_direction, Normal), 0.);
				Real pdf_cos_hemisphere = 2 * c_PI * (1. / (alpha + 1.)) * (1. / pow(cos_wo_normal, alpha)) / (Real)m_Vars.brdf_samples;
				ray.Direction = sample_direction;

				PDF_value = 1. / pdf_cos_hemisphere;

				color_weight += (surface_color * (material_phong.exponent + 1.) * (c_INVPI / 2.) * pow(cos_wo_normal, alpha)) * 
					pdf_cos_hemisphere;
			}
		}
	}
	else if (material.index() == 4) // blingphong
	{
		BlinnPhong& material_blinnphong = std::get<BlinnPhong>(material);

		int sqrt_N = sqrt(m_Vars.brdf_samples);
		Real u1;
		Real u2;
		for (int i = 0; i < sqrt_N; i++) {
			for (int j = 0; j < sqrt_N; j++) {
				Real u1_ori = next_pcg32_real<Real>(rng);
				Real u2_ori = next_pcg32_real<Real>(rng);
				u1 = (i + u1_ori) / sqrt_N;
				u2 = (j + u2_ori) / sqrt_N;

				Real alpha = material_blinnphong.exponent;
				Real theta = acos(pow(1. - u1, 1. / (alpha + 1)));
				Real phi = 2. * c_PI * u2;
				Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
				Vector3 z_axis = Normal;
				Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
				Vector3 y_axis = cross(z_axis, x_axis);
				Vector3 sample_direction = sample_direction_ori.x * x_axis +
					sample_direction_ori.y * y_axis +
					sample_direction_ori.z * z_axis;
				sample_direction = normalize(sample_direction);
				Vector3 half_vector = sample_direction;
				Vector3 w_o = reflect(ray.Direction, half_vector);
				Real cos_norm_half = dot(half_vector, Normal);
				Real cos_wo_half = dot(w_o, half_vector);

				surface_color = surface_color + pow(1. - cos_wo_half, 5.) * (Vector3(1., 1., 1.) - surface_color);

				if (dot(w_o, Normal) < 0)
				{
					PDF_value = 0.;
					continue;
				}

				Real pdf_inv = 8 * c_PI * cos_wo_half *
					(1. / ((alpha + 1.) * pow(cos_norm_half, alpha))) / (Real)m_Vars.brdf_samples;
				ray.Direction = w_o;

				PDF_value = 1. / pdf_inv;


				color_weight += surface_color * (alpha + 2.) * (c_INVPI / (4. * 2. - pow(2., -alpha / 2.))) * pow(cos_norm_half, alpha) *
					pdf_inv;
			}
		}
	}
	else if (material.index() == 5) // blingphongmicro
	{

		BlinnPhongMicrofacet& material_blinnphongMicro = std::get<BlinnPhongMicrofacet>(material);
		int sqrt_N = sqrt(m_Vars.brdf_samples);
		Real u1;
		Real u2;
		for (int i = 0; i < sqrt_N; i++) {
			for (int j = 0; j < sqrt_N; j++) {
				Real u1_ori = next_pcg32_real<Real>(rng);
				Real u2_ori = next_pcg32_real<Real>(rng);
				u1 = (i + u1_ori) / sqrt_N;
				u2 = (j + u2_ori) / sqrt_N;

				Real alpha = material_blinnphongMicro.exponent;
				Real theta = acos(pow(1. - u1, 1. / (alpha + 1)));
				Real phi = 2. * c_PI * u2;
				Vector3 sample_direction_ori = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
				Vector3 z_axis = Normal;
				Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
				Vector3 y_axis = cross(z_axis, x_axis);
				Vector3 sample_direction = sample_direction_ori.x * x_axis +
					sample_direction_ori.y * y_axis +
					sample_direction_ori.z * z_axis;
				sample_direction = normalize(sample_direction);

				Vector3 half_vector = sample_direction;
				Vector3 w_o = reflect(ray.Direction, half_vector);
				Real cos_norm_half = dot(half_vector, Normal);
				Real cos_wo_half = dot(w_o, half_vector);
				Real cos_wi_half = dot(-ray.Direction, half_vector);
				Real cos_wi_normal = dot(-ray.Direction, Normal);
				Real theta_i_normal = unit_angle(-ray.Direction, Normal);
				Real theta_o_normal = unit_angle(w_o, Normal);
				Real G_i = cos_wi_half > 0 ? GetGeoConfig(theta_i_normal, alpha) : 0.;
				Real G_o = cos_wo_half > 0 ? GetGeoConfig(theta_o_normal, alpha) : 0.;
				Real G_w = G_i * G_o;
				surface_color = surface_color + pow(1. - cos_wo_half, 5.) * (Vector3(1., 1., 1.) - surface_color);

				if (dot(w_o, Normal) < 0)
				{
					PDF_value = 0.;
					continue;
				}

				Real pdf_inv = 8 * c_PI * cos_wo_half *
					(1. / ((alpha + 1.) * pow(cos_norm_half, alpha))) / (Real)m_Vars.brdf_samples;

				ray.Direction = w_o;

				PDF_value = 1. / pdf_inv;

				color_weight += surface_color * (alpha + 2.) * (c_INVPI / 8.) * pow(cos_norm_half, alpha) * G_w * (1. / cos_wi_normal) *
					pdf_inv;
			}
		}
	}
	else if (material.index() == 6) // glass
	{
		Glass& material_glass = std::get<Glass>(material);
		Real index_of_refraction = material_glass.eta;

		if (!isFrontFace(ray, Normal))
		{
			index_of_refraction = 1. / material_glass.eta;
			Normal = -Normal;
		}

		double cos_theta = min(dot(-ray.Direction, Normal), 1.0);
		double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

		if (index_of_refraction * sin_theta > 1.0) {
			// Must Reflect
			ray.Direction = reflect(ray.Direction, Normal);
		}
		else {
			Real kr = reflectance(cos_theta, index_of_refraction);
			Real kt = 1 - kr;

			Ray refracted_ray = { ray.Origin, refract(ray.Direction, Normal, index_of_refraction) };
			// Can Refract
			ray.Direction = reflect(ray.Direction, Normal);

			//if (0 == max_depth--)
			//	return (this->*miss)(ray, scene, m_Vars, max_depth);
			//else
			//{
			//	return  color_weight + surface_color * (kt * TraceRay_BVH(refracted_ray, scene, true, max_depth, miss, illumination, rng)
			//		+ kr * TraceRay_BVH(ray, scene, true, max_depth, miss, illumination, rng));
			//}

		}

	}
	else if (material.index() == 7)
	{
		// Fuzzed
		ray.Direction = sample_in_unit_sphere;
		ray.Direction = normalize(ray.Direction);

		/*return color_weight;*/
	}

	return { Vector3(0., 0., 0.), color_weight , PDF_value};
}

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
Vector3 Renderer::Illumination_hw_1_1(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	Vector3 color(1., 0., 0.);
	return color;
}

Vector3 Renderer::Miss_hw_1_2(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return Vector3(0.5, 0.5, 0.5);
}
Vector3 Renderer::Illumination_hw_1_2(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	return (Normal + Vector3(1., 1., 1.)) / 2.;
}

Vector3 Renderer::Miss_hw_1_3(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return Vector3(0.5, 0.5, 0.5);
}
Vector3 Renderer::Illumination_hw_1_3(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	return (Normal + Vector3(1., 1., 1.)) / 2.;
}

Vector3 Renderer::Miss_hw_1_4(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return Vector3(0.5, 0.5, 0.5);
}
Vector3 Renderer::Illumination_hw_1_4(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 color = std::get<Vector3>(m_color);
	return color;
}

Vector3 Renderer::Miss_hw_1_5(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return Vector3(0.5, 0.5, 0.5);
}
Vector3 Renderer::Illumination_hw_1_5(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}
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

		if (!Renderer::FindNearstIntersection(shadow_ray, scene, 1e-4, d, rng))
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
	Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	return Illumination_hw_1_5(vis_ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars, rng);
}

Vector3 Renderer::Miss_hw_1_7(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	//if (*remain_reflect > 0)
	//	return Vector3(0., 0., 0.);
	return Vector3(0.5, 0.5, 0.5);
}

Vector3 Renderer::Illumination_hw_1_7(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{

	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}
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
		if (!Renderer::FindNearstIntersection(shadow_ray, scene, 1e-4, d, rng))
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
Vector3 Renderer::Illumination_hw_1_8(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	return Illumination_hw_1_7(ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars, rng);
}




Vector3 Renderer::Miss_hw_1_10(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	
	auto t = (-ray.Direction.y + 2.0) * 0.73;
	auto v = (ray.Direction.z ) / 2;
	return (t*t + v) * Vector3(0.6, 0.9, 0.95);
	//return Vector3(0.5, 0.5, 0.5);
}

Vector3 Renderer::Illumination_hw_1_10(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{

	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}

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
		if (!Renderer::FindNearstIntersection(shadow_ray, scene, 1e-4, d, rng))
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
		else if (material.index() == 6) // glass
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

Vector3 Renderer::Illumination_hw_1_11(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	return Illumination_hw_1_10(ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars, rng);
}

Vector3 Renderer::Miss_hw_1_9(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{

	auto t = (-ray.Direction.y + 2.0) * 0.73;
	auto v = (ray.Direction.z) / 2;
	return (t * t + v) * Vector3(0.6, 0.9, 0.95);
	//return Vector3{ 0.2, 0.2, 0.2};
	//return Miss_hw_1_10(ray, scene, vars, max_depth);
}

Vector3 Renderer::Illumination_hw_1_9(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	return Illumination_hw_1_10(ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars, rng);
}


Vector3 Renderer::Miss_hw_2_1(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{

	//auto t = (-ray.Direction.y + 2.0) * 0.73;
	//auto v = (ray.Direction.z) / 2;
	//return (t * t + v) * Vector3(0.6, 0.9, 0.95);
	return Vector3(0.5, 0.5, 0.5);
}

Vector3 Renderer::Illumination_hw_2_1(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
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

Vector3 Renderer::Illumination_hw_2_2(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	return Illumination_hw_2_1(ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars, rng);
}

Vector3 Renderer::Miss_hw_2_3(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_2_3(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	return Illumination_hw_1_10(ray, isPrimary_ray, HitPoint, Normal, NearstObj, scene, vars, rng);
}

Vector3 Renderer::Miss_hw_2_4(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_2_4(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	return Vector3(1., 1., 1.);
}

Vector3 Renderer::Miss_hw_2_5(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_2_5(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}

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
		if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, d, rng))
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
		else if (material.index() == 6) // glass
			color += transmission * visibility * c_INVPI;
	}

	//color += ambient + emission;

	return color;
}

Vector3 Renderer::Miss_hw_3_1(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_3_1(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if(m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}
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
		if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, d, rng))
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
		else if (material.index() == 6) // glass
			color += transmission * visibility * c_INVPI;
	}

	//color += ambient + emission;

	return color;
}

Vector3 Renderer::Miss_hw_3_3(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_3_3(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}
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
		if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, d, rng))
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
		else if(material.index() == 2) // plastic
		{
			Plastic & material_plastic = std::get<Plastic>(material);
			Real index_of_refraction = material_plastic.eta;
			Real cos_theta = dot(reflect(ray.Direction, normal), normal);
			Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
			Real F = F_0 + (1 - F_0) * pow(1 - cos_theta, 5);
			color += (1. - F) * diffuse * visibility * c_INVPI;
		}
		else if (material.index() == 6) // glass
			color += transmission * visibility * c_INVPI;
	}

	//color += ambient + emission;

	return color;
}

Vector3 Renderer::Miss_hw_3_4(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_3_4(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}


	Vector3 color{ 0., 0., 0. };
	//parallel_for([&](int counts)
	//	{
	//	}, (int64_t)10);
	for (ParsedLight light : scene.lights)
	{
		std::vector<Real> d;
		std::vector<Real> visibility;
		std::vector<Real> pdf_and_cos;
		std::vector<Vector3> position;
		std::vector<Vector3> intensity;
		Vector3 light_direction = Vector3(0., 0., 0.);
		if (light.index() == 0) // is PointLight
		{
			ParsedPointLight point_light = std::get<ParsedPointLight>(light);
			position.push_back(point_light.position);
			intensity.push_back(point_light.intensity);
			pdf_and_cos.push_back(1.);
			light_direction = HitPoint - point_light.position;
			d.push_back(length(light_direction));
			Ray shadow_ray = { HitPoint, -normalize(light_direction) };
			if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
				visibility.push_back(1.);
			else
				visibility.push_back(0);
		}
		else // is area light
		{
			ParsedDiffuseAreaLight area_light = std::get<ParsedDiffuseAreaLight>(light);
			if (scene.src_shapes[area_light.shape_id]->index() == 0) // sphere
			{
				Real u1 = next_pcg32_real<Real>(rng);
				Real u2 = next_pcg32_real<Real>(rng);
				ParsedSphere& light_sphere = std::get<ParsedSphere>(*scene.src_shapes[area_light.shape_id]);
				Real theta = acos(1. - 2. * u1);
				Real phi = 2. * c_PI * u2;
				Vector3 offset = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
				offset *= light_sphere.radius;
				Vector3 hit_p = light_sphere.position + offset;
				position.push_back(hit_p);
				light_direction = HitPoint - hit_p;
				d.push_back(length(light_direction));
				intensity.push_back(area_light.radiance);
				Ray shadow_ray = { HitPoint, -normalize(light_direction) };
				if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
					visibility.push_back(1.);
				else
				{
					int shadow_hit_area_light_id = std::visit([&](auto shape) {return shape.area_light_id; }, *shadow_ray.object);
					if(shadow_hit_area_light_id == light_sphere.area_light_id)
						visibility.push_back(1.);
					else
						visibility.push_back(0);
				}

				light_direction = normalize(light_direction);
				Vector4 normal_original = GetNormalByHitPoint(offset, light_sphere);
				Vector4 normal_transformed_4 = Vector4(transpose(inverse(
					translate(Vector3(light_sphere.position.x, light_sphere.position.y, light_sphere.position.z)))) * normal_original);
				Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
				Real cos_light = max(dot(light_direction, normal_transformed), 0.);
				pdf_and_cos.push_back(cos_light * 4 * c_PI * light_sphere.radius * light_sphere.radius);

				

			}
			else // trianglemesh
			{
				ParsedTriangleMesh& light_mesh = std::get<ParsedTriangleMesh>(*scene.src_shapes[area_light.shape_id]);
				for(int i = 0; i < light_mesh.indices.size(); i++)
				{
					Real u1 = next_pcg32_real<Real>(rng);
					Real u2 = next_pcg32_real<Real>(rng);
					//std::cout << "u1:" << u1 << ",u2:" << u2 << std::endl;
					Triangle light_triangle = {&light_mesh, i, light_mesh.area_light_id};
					Real b1 = 1. - sqrt(u1);
					Real b2 = u2 * sqrt(u1);
					//std::cout << "b1:" << b1 << ",b2:" << b2 << std::endl;
					Vector3 p1 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][0]];
					Vector3 p2 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][1]];
					Vector3 p3 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][2]];
					Vector3 hit_p = (1. - b1 - b2)* p1 + b1 * p2 + b2 * p3;
					position.push_back(hit_p);
					light_direction = HitPoint - hit_p;
					d.push_back(length(light_direction));
					intensity.push_back(area_light.radiance);
					Ray shadow_ray = { HitPoint, -normalize(light_direction) };
					if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
						visibility.push_back(1.);
					else
					{
						int shadow_hit_area_light_id = std::visit([&](auto shape) {return shape.area_light_id; }, *shadow_ray.object);
						if (shadow_hit_area_light_id == light_mesh.area_light_id)
							visibility.push_back(1.);
						else
							visibility.push_back(0);
					}

					light_direction = normalize(light_direction);
					Vector4 normal_original = GetNormalByHitPoint(hit_p, light_triangle);
					Vector4 normal_transformed_4 = normal_original;
					Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
					Real cos_light = max(dot(light_direction, normal_transformed), 0.);
					pdf_and_cos.push_back(cos_light * GetAreaByShape(light_triangle));
				}

			}
		}


		//For rainbow color
		//light->light_color = vec4(sin((ray.Origin.x + 10.) / (12.7 / 1.5)), sin((ray.Origin.z + 50.) / (31.8 / 1.5)), cos((ray.Origin.x + 10.) / (12.7 / 1.5)), 1.);

		Vector3 diffuse = Vector3(0., 0., 0.);
		Vector3 normal = normalize(Normal);

		if (dot(ray.Direction, normal) > 0)
		{
			normal = -normal;
		}

		for (int i = 0; i < position.size(); i++)
		{
			light_direction = normalize(HitPoint - position[i]);
			diffuse += intensity[i] * surface_color *
				max(dot(-light_direction, normal), 0.) * pdf_and_cos[i] / 
				(vars.attenuation_const + vars.attenuation_linear * d[i] + vars.attenuation_quadratic * d[i] * d[i]);


			//Vector3 halfvec = normalize(-direction - light_direction);
			//Vector3 specular = intensity * surface_color *
			//	pow(max(dot(halfvec, normalize(normal)), 0.), 1000000);

			Vector3 transmission{ 0., 0., 0. };

			//specular = specular / (vars.attenuation_const + vars.attenuation_linear * d + vars.attenuation_quadratic * d * d);
			if (material.index() == 0)// diffuse
				color += diffuse * visibility[i] * c_INVPI;
			else if (material.index() == 1) // mirror
				//color += specular * visibility * c_INVPI;	
				;
			else if (material.index() == 2) // plastic
			{
				Plastic& material_plastic = std::get<Plastic>(material);
				Real index_of_refraction = material_plastic.eta;
				Real cos_theta = dot(reflect(ray.Direction, normal), normal);
				Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
				Real F = F_0 + (1 - F_0) * pow(1 - cos_theta, 5);
				color += (1. - F) * diffuse * visibility[i] * c_INVPI;
			}
			else if (material.index() == 6) // glass
				color += transmission * visibility[i] * c_INVPI;
		}

	}

	//color += ambient + emission;

	return color;
}

Vector3 Renderer::Miss_hw_3_7(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_3_7(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}


	Vector3 color{ 0., 0., 0. };
	//parallel_for([&](int counts)
	//	{
	//	}, (int64_t)10);
	for (ParsedLight light : scene.lights)
	{
		std::vector<Real> d;
		std::vector<Real> visibility;
		std::vector<Real> pdf_and_cos;
		std::vector<Vector3> position;
		std::vector<Vector3> intensity;
		Vector3 light_direction = Vector3(0., 0., 0.);
		if (light.index() == 0) // is PointLight
		{
			ParsedPointLight point_light = std::get<ParsedPointLight>(light);
			position.push_back(point_light.position);
			intensity.push_back(point_light.intensity);
			pdf_and_cos.push_back(1.);
			light_direction = HitPoint - point_light.position;
			d.push_back(length(light_direction));
			Ray shadow_ray = { HitPoint, -normalize(light_direction) };
			if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
				visibility.push_back(1.);
			else
				visibility.push_back(0);
		}
		else // is area light
		{
			ParsedDiffuseAreaLight area_light = std::get<ParsedDiffuseAreaLight>(light);
			if (scene.src_shapes[area_light.shape_id]->index() == 0) // sphere
			{
				int sqrt_N = sqrt(vars.area_light_samples);
				for (int i = 0; i < sqrt_N; i++) {
					for (int j = 0; j < sqrt_N; j++) {
						Real u1 = next_pcg32_real<Real>(rng);
						Real u2 = next_pcg32_real<Real>(rng);

						ParsedSphere& light_sphere = std::get<ParsedSphere>(*scene.src_shapes[area_light.shape_id]);
						Real theta = acos(1. - 2. * u1);
						Real phi = 2. * c_PI * u2;
						Vector3 offset = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
						offset *= light_sphere.radius;
						Vector3 hit_p = light_sphere.position + offset;
						position.push_back(hit_p);
						light_direction = HitPoint - hit_p;
						d.push_back(length(light_direction));
						intensity.push_back(area_light.radiance);
						Ray shadow_ray = { HitPoint, -normalize(light_direction) };
						if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
							visibility.push_back(1.);
						else
						{
							int shadow_hit_area_light_id = std::visit([&](auto shape) {return shape.area_light_id; }, *shadow_ray.object);
							if (shadow_hit_area_light_id == light_sphere.area_light_id)
								visibility.push_back(1.);
							else
								visibility.push_back(0);
						}

						light_direction = normalize(light_direction);
						Vector4 normal_original = GetNormalByHitPoint(offset, light_sphere);
						Vector4 normal_transformed_4 = Vector4(transpose(inverse(
							translate(Vector3(light_sphere.position.x, light_sphere.position.y, light_sphere.position.z)))) * normal_original);
						Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
						Real cos_light = max(dot(light_direction, normal_transformed), 0.);
						pdf_and_cos.push_back(cos_light * 4 * c_PI * light_sphere.radius * light_sphere.radius / (Real)vars.area_light_samples);
					}
				}
			}
			else // trianglemesh
			{
				ParsedTriangleMesh& light_mesh = std::get<ParsedTriangleMesh>(*scene.src_shapes[area_light.shape_id]);
				for (int i = 0; i < light_mesh.indices.size(); i++)
				{
					int sqrt_N = sqrt(vars.area_light_samples);
					for (int k = 0; k < sqrt_N; k++) {
						for (int j = 0; j < sqrt_N; j++) {
							Real u1 = next_pcg32_real<Real>(rng);
							Real u2 = next_pcg32_real<Real>(rng);
							//std::cout << "u1:" << u1 << ",u2:" << u2 << std::endl;
							Triangle light_triangle = { &light_mesh, i, light_mesh.area_light_id };
							Real b1 = 1. - sqrt(u1);
							Real b2 = u2 * sqrt(u1);
							//std::cout << "b1:" << b1 << ",b2:" << b2 << std::endl;
							Vector3 p1 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][0]];
							Vector3 p2 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][1]];
							Vector3 p3 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][2]];
							Vector3 hit_p = (1. - b1 - b2) * p1 + b1 * p2 + b2 * p3;
							position.push_back(hit_p);
							light_direction = HitPoint - hit_p;
							d.push_back(length(light_direction));
							intensity.push_back(area_light.radiance);
							Ray shadow_ray = { HitPoint, -normalize(light_direction) };
							if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
								visibility.push_back(1.);
							else
							{
								int shadow_hit_area_light_id = std::visit([&](auto shape) {return shape.area_light_id; }, *shadow_ray.object);
								if (shadow_hit_area_light_id == light_mesh.area_light_id)
									visibility.push_back(1.);
								else
									visibility.push_back(0);
							}

							light_direction = normalize(light_direction);
							Vector4 normal_original = GetNormalByHitPoint(hit_p, light_triangle);
							Vector4 normal_transformed_4 = normal_original;
							Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
							Real cos_light = max(dot(light_direction, normal_transformed), 0.);
							pdf_and_cos.push_back(cos_light * GetAreaByShape(light_triangle) / (Real)vars.area_light_samples);
						}
					}
				}
			}
		}


		//For rainbow color
		//light->light_color = vec4(sin((ray.Origin.x + 10.) / (12.7 / 1.5)), sin((ray.Origin.z + 50.) / (31.8 / 1.5)), cos((ray.Origin.x + 10.) / (12.7 / 1.5)), 1.);

		Vector3 diffuse = Vector3(0., 0., 0.);
		Vector3 normal = normalize(Normal);

		if (dot(ray.Direction, normal) > 0)
		{
			normal = -normal;
		}

		for (int i = 0; i < position.size(); i++)
		{
			light_direction = normalize(HitPoint - position[i]);
			diffuse += intensity[i] * surface_color *
				max(dot(-light_direction, normal), 0.) * pdf_and_cos[i] /
				(vars.attenuation_const + vars.attenuation_linear * d[i] + vars.attenuation_quadratic * d[i] * d[i]);


			//Vector3 halfvec = normalize(-direction - light_direction);
			//Vector3 specular = intensity * surface_color *
			//	pow(max(dot(halfvec, normalize(normal)), 0.), 1000000);

			Vector3 transmission{ 0., 0., 0. };

			//specular = specular / (vars.attenuation_const + vars.attenuation_linear * d + vars.attenuation_quadratic * d * d);
			if (material.index() == 0)// diffuse
				color += diffuse * visibility[i] * c_INVPI;
			else if (material.index() == 1) // mirror
				//color += specular * visibility * c_INVPI;	
				;
			else if (material.index() == 2) // plastic
			{
				Plastic& material_plastic = std::get<Plastic>(material);
				Real index_of_refraction = material_plastic.eta;
				Real cos_theta = dot(reflect(ray.Direction, normal), normal);
				Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
				Real F = F_0 + (1 - F_0) * pow(1 - cos_theta, 5);
				color += (1. - F) * diffuse * visibility[i] * c_INVPI;
			}
			else if (material.index() == 6) // glass
				color += transmission * visibility[i] * c_INVPI;
			else if (material.index() == 7) // volume
			{
				//color += diffuse * visibility[i] * c_INVPI * max(dot(-light_direction, Vector3(0., 0., 1.)), 0.);
			}
		}

	}


	//color += ambient + emission;

	return color;
}

Vector3 Renderer::Miss_hw_3_9(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_3_9(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}


	Vector3 color{ 0., 0., 0. };
	//parallel_for([&](int counts)
	//	{
	//	}, (int64_t)10);
	for (ParsedLight light : scene.lights)
	{
		std::vector<Real> d;
		std::vector<Real> visibility;
		std::vector<Real> pdf_and_cos;
		std::vector<Vector3> position;
		std::vector<Vector3> intensity;
		Vector3 light_direction = Vector3(0., 0., 0.);
		if (light.index() == 0) // is PointLight
		{
			ParsedPointLight point_light = std::get<ParsedPointLight>(light);
			position.push_back(point_light.position);
			intensity.push_back(point_light.intensity);
			pdf_and_cos.push_back(1.);
			light_direction = HitPoint - point_light.position;
			d.push_back(length(light_direction));
			Ray shadow_ray = { HitPoint, -normalize(light_direction) };
			if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
				visibility.push_back(1.);
			else
				visibility.push_back(0);
		}
		else // is area light
		{
			ParsedDiffuseAreaLight area_light = std::get<ParsedDiffuseAreaLight>(light);
			if (scene.src_shapes[area_light.shape_id]->index() == 0) // sphere
			{
				int sqrt_N = sqrt(vars.area_light_samples);
				Real u1;
				Real u2;
				for (int i = 0; i < sqrt_N; i++) {
					for (int j = 0; j < sqrt_N; j++) {
						Real u1_ori = next_pcg32_real<Real>(rng);
						Real u2_ori = next_pcg32_real<Real>(rng);
						u1 = (i + u1_ori) / sqrt_N;
						u2 = (j + u2_ori) / sqrt_N;

						ParsedSphere& light_sphere = std::get<ParsedSphere>(*scene.src_shapes[area_light.shape_id]);
						Real theta = acos(1. - 2. * u1);
						Real phi = 2. * c_PI * u2;
						Vector3 offset = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };
						offset *= light_sphere.radius;
						Vector3 hit_p = light_sphere.position + offset;
						position.push_back(hit_p);
						light_direction = HitPoint - hit_p;
						d.push_back(length(light_direction));
						intensity.push_back(area_light.radiance);
						Ray shadow_ray = { HitPoint, -normalize(light_direction) };
						if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
							visibility.push_back(1.);
						else
						{
							int shadow_hit_area_light_id = std::visit([&](auto shape) {return shape.area_light_id; }, *shadow_ray.object);
							if (shadow_hit_area_light_id == light_sphere.area_light_id)
								visibility.push_back(1.);
							else
								visibility.push_back(0);
						}

						light_direction = normalize(light_direction);
						Vector4 normal_original = GetNormalByHitPoint(offset, light_sphere);
						Vector4 normal_transformed_4 = Vector4(transpose(inverse(
							translate(Vector3(light_sphere.position.x, light_sphere.position.y, light_sphere.position.z)))) * normal_original);
						Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
						Real cos_light = max(dot(light_direction, normal_transformed), 0.);
						pdf_and_cos.push_back(cos_light * 4 * c_PI * light_sphere.radius * light_sphere.radius / (Real)vars.area_light_samples);
					}
				}
			}
			else // trianglemesh
			{
				ParsedTriangleMesh& light_mesh = std::get<ParsedTriangleMesh>(*scene.src_shapes[area_light.shape_id]);
				for (int i = 0; i < light_mesh.indices.size(); i++)
				{
					int sqrt_N = sqrt(vars.area_light_samples);
					Real u1;
					Real u2;
					for (int k = 0; k < sqrt_N; k++) {
						for (int j = 0; j < sqrt_N; j++) {
							Real u1_ori = next_pcg32_real<Real>(rng);
							Real u2_ori = next_pcg32_real<Real>(rng);
							u1 = (k + u1_ori) / sqrt_N;
							u2 = (j + u2_ori) / sqrt_N;
							//std::cout << "u1:" << u1 << ",u2:" << u2 << std::endl;
							Triangle light_triangle = { &light_mesh, i, light_mesh.area_light_id };
							Real b1 = 1. - sqrt(u1);
							Real b2 = u2 * sqrt(u1);
							//std::cout << "b1:" << b1 << ",b2:" << b2 << std::endl;
							Vector3 p1 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][0]];
							Vector3 p2 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][1]];
							Vector3 p3 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][2]];
							Vector3 hit_p = (1. - b1 - b2) * p1 + b1 * p2 + b2 * p3;
							position.push_back(hit_p);
							light_direction = HitPoint - hit_p;
							d.push_back(length(light_direction));
							intensity.push_back(area_light.radiance);
							Ray shadow_ray = { HitPoint, -normalize(light_direction) };
							if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
								visibility.push_back(1.);
							else
							{
								int shadow_hit_area_light_id = std::visit([&](auto shape) {return shape.area_light_id; }, *shadow_ray.object);
								if (shadow_hit_area_light_id == light_mesh.area_light_id)
									visibility.push_back(1.);
								else
									visibility.push_back(0);
							}

							light_direction = normalize(light_direction);
							Vector4 normal_original = GetNormalByHitPoint(hit_p, light_triangle);
							Vector4 normal_transformed_4 = normal_original;
							Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
							Real cos_light = max(dot(light_direction, normal_transformed), 0.);
							pdf_and_cos.push_back(cos_light * GetAreaByShape(light_triangle) / (Real)vars.area_light_samples);
						}
					}
				}
			}
		}


		//For rainbow color
		//light->light_color = vec4(sin((ray.Origin.x + 10.) / (12.7 / 1.5)), sin((ray.Origin.z + 50.) / (31.8 / 1.5)), cos((ray.Origin.x + 10.) / (12.7 / 1.5)), 1.);

		Vector3 diffuse = Vector3(0., 0., 0.);
		Vector3 normal = normalize(Normal);

		if (dot(ray.Direction, normal) > 0)
		{
			normal = -normal;
		}

		for (int i = 0; i < position.size(); i++)
		{
			light_direction = normalize(HitPoint - position[i]);
			diffuse += intensity[i] * surface_color *
				max(dot(-light_direction, normal), 0.) * pdf_and_cos[i] / 
				(vars.attenuation_const + vars.attenuation_linear * d[i] + vars.attenuation_quadratic * d[i] * d[i]);


			//Vector3 halfvec = normalize(-direction - light_direction);
			//Vector3 specular = intensity * surface_color *
			//	pow(max(dot(halfvec, normalize(normal)), 0.), 1000000);

			Vector3 transmission{ 0., 0., 0. };

			//specular = specular / (vars.attenuation_const + vars.attenuation_linear * d + vars.attenuation_quadratic * d * d);
			if (material.index() == 0)// diffuse
				color += diffuse * visibility[i] * c_INVPI;
			else if (material.index() == 1) // mirror
				//color += specular * visibility * c_INVPI;	
				;
			else if (material.index() == 2) // plastic
			{
				Plastic& material_plastic = std::get<Plastic>(material);
				Real index_of_refraction = material_plastic.eta;
				Real cos_theta = dot(reflect(ray.Direction, normal), normal);
				Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
				Real F = F_0 + (1 - F_0) * pow(1 - cos_theta, 5);
				color += (1. - F) * diffuse * visibility[i] * c_INVPI;
			}
			else if (material.index() == 6) // glass
				color += transmission * visibility[i] * c_INVPI;
			else if (material.index() == 7) // volume
			{
				//color += diffuse * visibility[i] * c_INVPI * max(dot(-light_direction, Vector3(0., 0., 1.)), 0.);
			}
		}

	}

	//color += ambient + emission;

	return color;
}

Vector3 Renderer::Miss_hw_3_11(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return Vector3(0., 0., 0.);
}

Vector3 Renderer::Illumination_hw_3_11(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}


	Vector3 color{ 0., 0., 0. };
	//parallel_for([&](int counts)
	//	{
	//	}, (int64_t)10);
	for (ParsedLight light : scene.lights)
	{
		std::vector<Real> d;
		std::vector<Real> visibility;
		std::vector<Real> pdf_and_cos;
		std::vector<Vector3> position;
		std::vector<Vector3> intensity;
		Vector3 light_direction = Vector3(0., 0., 0.);
		if (light.index() == 0) // is PointLight
		{
			ParsedPointLight point_light = std::get<ParsedPointLight>(light);
			position.push_back(point_light.position);
			intensity.push_back(point_light.intensity);
			pdf_and_cos.push_back(1.);
			light_direction = HitPoint - point_light.position;
			d.push_back(length(light_direction));
			Ray shadow_ray = { HitPoint, -normalize(light_direction) };
			if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
				visibility.push_back(1.);
			else
				visibility.push_back(0);
		}
		else // is area light
		{
			ParsedDiffuseAreaLight area_light = std::get<ParsedDiffuseAreaLight>(light);
			if (scene.src_shapes[area_light.shape_id]->index() == 0) // sphere
			{
				int sqrt_N = sqrt(vars.area_light_samples);
				Real u1;
				Real u2;
				for (int i = 0; i < sqrt_N; i++) {
					for (int j = 0; j < sqrt_N; j++) {
						Real u1_ori = next_pcg32_real<Real>(rng);
						Real u2_ori = next_pcg32_real<Real>(rng);
						u1 = (i + u1_ori) / sqrt_N;
						u2 = (j + u2_ori) / sqrt_N;

						ParsedSphere& light_sphere = std::get<ParsedSphere>(*scene.src_shapes[area_light.shape_id]);
						Real dc = length(light_sphere.position - HitPoint);
						Real cos_theta_max = sqrt(max(0., (light_sphere.radius * light_sphere.radius) / dc * dc));

						Real theta = acos(1. + (cos_theta_max - 1.) * u1);
						//std::cout << "theta:" << theta << std::endl;
						//Real theta = acos(1. - 2. * u1);
						Real phi = 2. * c_PI * u2;
						Real r = light_sphere.radius;
						Real cos_theta = cos(theta);
						Real sin_theta = sin(theta);
						Vector3 offset = { sin_theta * cos(phi), sin_theta * sin(phi), cos_theta };
						Vector3 z_axis = normalize(HitPoint - light_sphere.position);
						Vector3 x_axis = normalize(cross(z_axis, z_axis - Vector3(0.1, 0.1, 0.1)));
						Vector3 y_axis = cross(z_axis, x_axis);
						Vector3 sample_direction = offset.x * x_axis +
							offset.y * y_axis +
							offset.z * z_axis;
						sample_direction = normalize(sample_direction);
						offset = sample_direction * light_sphere.radius;
						Vector3 hit_p = light_sphere.position + offset;
						position.push_back(hit_p);
						light_direction = HitPoint - hit_p;
						d.push_back(length(light_direction));
						intensity.push_back(area_light.radiance);
						Ray shadow_ray = { HitPoint, -normalize(light_direction) };
						if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
							visibility.push_back(1.);
						else
						{
							int shadow_hit_area_light_id = std::visit([&](auto shape) {return shape.area_light_id; }, *shadow_ray.object);
							if (shadow_hit_area_light_id == light_sphere.area_light_id)
								visibility.push_back(1.);
							else
								visibility.push_back(0);
						}

						light_direction = normalize(light_direction);
						Vector4 normal_original = GetNormalByHitPoint(offset, light_sphere);
						Vector4 normal_transformed_4 = Vector4(transpose(inverse(
							translate(normalize(Vector3(light_sphere.position.x, light_sphere.position.y, light_sphere.position.z))))) * normal_original);
						Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
						Real cos_light = max(dot(light_direction, normal_transformed), 0.);
						pdf_and_cos.push_back(cos_light * 2. * (1. - cos_theta_max) * c_PI * light_sphere.radius * light_sphere.radius / (Real) vars.area_light_samples);
					}
				}
			}
			else // trianglemesh
			{
				ParsedTriangleMesh& light_mesh = std::get<ParsedTriangleMesh>(*scene.src_shapes[area_light.shape_id]);
				for (int i = 0; i < light_mesh.indices.size(); i++)
				{
					if(light_mesh.indices.size() > 100)
					{
						if (i % light_mesh.indices.size() != 0)
							continue;
					}
					int sqrt_N = sqrt(vars.area_light_samples);
					Real u1;
					Real u2;
					for (int k = 0; k < sqrt_N; k++) {
						for (int j = 0; j < sqrt_N; j++) {
							Real u1_ori = next_pcg32_real<Real>(rng);
							Real u2_ori = next_pcg32_real<Real>(rng);
							u1 = (k + u1_ori) / sqrt_N;
							u2 = (j + u2_ori) / sqrt_N;
							//std::cout << "u1:" << u1 << ",u2:" << u2 << std::endl;
							Triangle light_triangle = { &light_mesh, i, light_mesh.area_light_id };
							Real b1 = 1. - sqrt(u1);
							Real b2 = u2 * sqrt(u1);
							//std::cout << "b1:" << b1 << ",b2:" << b2 << std::endl;
							Vector3 p1 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][0]];
							Vector3 p2 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][1]];
							Vector3 p3 = light_triangle.mesh->positions[light_triangle.mesh->indices[light_triangle.index][2]];
							Vector3 hit_p = (1. - b1 - b2) * p1 + b1 * p2 + b2 * p3;
							position.push_back(hit_p);
							light_direction = HitPoint - hit_p;
							d.push_back(length(light_direction));
							intensity.push_back(area_light.radiance);
							Ray shadow_ray = { HitPoint, -normalize(light_direction) };
							if (!FindNearstIntersection_BVH(shadow_ray, scene, m_BVH, 1e-4, length(light_direction), rng))
								visibility.push_back(1.);
							else
							{
								int shadow_hit_area_light_id = std::visit([&](auto shape) {return shape.area_light_id; }, *shadow_ray.object);
								if (shadow_hit_area_light_id == light_mesh.area_light_id)
									visibility.push_back(1.);
								else
									visibility.push_back(0);
							}

							light_direction = normalize(light_direction);
							Vector4 normal_original = GetNormalByHitPoint(hit_p, light_triangle);
							Vector4 normal_transformed_4 = normal_original;
							Vector3 normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
							Real cos_light = max(dot(light_direction, normal_transformed), 0.);
							pdf_and_cos.push_back(cos_light * GetAreaByShape(light_triangle) / (Real)vars.area_light_samples);
						}
					}
				}
			}
		}


		//For rainbow color
		//light->light_color = vec4(sin((ray.Origin.x + 10.) / (12.7 / 1.5)), sin((ray.Origin.z + 50.) / (31.8 / 1.5)), cos((ray.Origin.x + 10.) / (12.7 / 1.5)), 1.);

		Vector3 diffuse = Vector3(0., 0., 0.);
		Vector3 normal = normalize(Normal);

		if (dot(ray.Direction, normal) > 0)
		{
			normal = -normal;
		}

		for (int i = 0; i < position.size(); i++)
		{
			light_direction = normalize(HitPoint - position[i]);
			diffuse += intensity[i] * surface_color *
				max(dot(-light_direction, normal), 0.) * pdf_and_cos[i] / 
				(vars.attenuation_const + vars.attenuation_linear * d[i] + vars.attenuation_quadratic * d[i] * d[i]);


			//Vector3 halfvec = normalize(-direction - light_direction);
			//Vector3 specular = intensity * surface_color *
			//	pow(max(dot(halfvec, normalize(normal)), 0.), 1000000);

			Vector3 transmission{ 0., 0., 0. };

			if (material.index() == 0)// diffuse
				color += diffuse * visibility[i] * c_INVPI;
			else if (material.index() == 1) // mirror
				//color += specular * visibility * c_INVPI;	
				;
			else if (material.index() == 2) // plastic
			{
				Plastic& material_plastic = std::get<Plastic>(material);
				Real index_of_refraction = material_plastic.eta;
				Real cos_theta = dot(reflect(ray.Direction, normal), normal);
				Real F_0 = pow((index_of_refraction - 1.) / (index_of_refraction + 1.), 2.);
				Real F = F_0 + (1 - F_0) * pow(1 - cos_theta, 5);
				color += (1. - F) * diffuse * visibility[i] * c_INVPI;
			}
			else if (material.index() == 6) // glass
				color += transmission * visibility[i] * c_INVPI;
			else if (material.index() == 7) // volume
			{
				//color += diffuse * visibility[i] * c_INVPI * max(dot(-light_direction, Vector3(0., 0., 1.)), 0.);
			}
		}

	}

	//color += ambient + emission;

	return color;
}


Vector3 Renderer::Miss_hw_4_1(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_4_1(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}

	Vector3 color{ 0., 0., 0. };
	int area_light_id = std::visit([&](auto& shape) {return shape.area_light_id; }, NearstObj);
	if (area_light_id != -1) // is area_light
	{
		Vector3 l_e = std::get<ParsedDiffuseAreaLight>(scene.lights[area_light_id]).radiance;
		color = l_e;
	}
	return color;
}

Vector3 Renderer::Miss_hw_4_2(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_4_2(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}

	Vector3 color{ 0., 0., 0. };
	int area_light_id = std::visit([&](auto& shape) {return shape.area_light_id; }, NearstObj);
	if (area_light_id != -1 && dot(Normal, -ray.Direction)) // is area_light
	{
		Vector3 l_e = std::get<ParsedDiffuseAreaLight>(scene.lights[area_light_id]).radiance;
		color = l_e;
	}
	return color;
}

Vector3 Renderer::Miss_hw_4_3(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	return scene.background_color;
}

Vector3 Renderer::Illumination_hw_4_3(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}

	Vector3 color{ 0., 0., 0. };
	int area_light_id = std::visit([&](auto& shape) {return shape.area_light_id; }, NearstObj);

	if (area_light_id != -1 && dot(Normal, -ray.Direction) > 0) // is area_light
	{
		Vector3 l_e = std::get<ParsedDiffuseAreaLight>(scene.lights[area_light_id]).radiance;
		color = l_e;
	}
	return color;
}

Vector3 Renderer::Miss_final(const Ray& ray, Scene& scene, Variables& vars, int max_depth)
{
	for(ParsedLight light : scene.lights)
	{
		if (light.index() == 2) // env_map
		{
			ParsedEnvMap& envmap = std::get<ParsedEnvMap>(light);
			Vector4 direc_light_4 = envmap.world_to_light * Vector4(ray.Direction, 0.);
			Vector3 direc_light = normalize(Vector3(direc_light_4.x, direc_light_4.y, direc_light_4.z));
			Vector2 uv_real = envmap.GetUV(direc_light);
			Vector2i uv_int = { (int)(uv_real[0] * (Real)envmap.texture.width),
								(int)(uv_real[1] * (Real)envmap.texture.height) };
			Real pdf = envmap.dist_2d.GetPDF(uv_int);
			// dudw * dwdx = dudx
			Real theta = acos(direc_light.y);
			Real phi = atan2(-direc_light.z, direc_light.x) + c_PI;
			Real footprint = (abs(1. / max(sin(theta), 1e-9)) / ((Real)envmap.texture.width * 2. * c_PI)
				+ 1. / ((Real)envmap.texture.height * c_PI)) / 2.;
			Real level = envmap.texture.GetLevel(footprint);
			Vector3 color = envmap.texture.GetColor(uv_real[0], uv_real[1], level);
			//Vector3 luminance(pdf, pdf, pdf);
			//return luminance * envmap.intensity_scale;
			return color * envmap.intensity_scale;
		}
	}
	return scene.background_color;
}

Vector3 Renderer::Illumination_final(Ray& ray, bool isPrimary_ray, Vector3 HitPoint, Vector3 Normal, Shape& NearstObj, Scene& scene, Variables& vars, pcg32_state& rng)
{
	int material_id;
	if (NearstObj.index() == 0) // sphere
		material_id = std::get<ParsedSphere>(NearstObj).material_id;
	else // triangle
		material_id = std::get<Triangle>(NearstObj).mesh->material_id;
	Material material = scene.materials[material_id];
	Color m_color = std::visit([=](auto& m) {return m.reflectance; }, material);
	Vector3 surface_color;
	if (m_color.index() == 0) // RGB color
		surface_color = std::get<Vector3>(m_color);
	else // Texture
	{
		Texture& texture = std::get<Texture>(m_color);
		surface_color = texture.GetColor(ray.u_coor, ray.v_coor);
	}

	Vector3 color{ 0., 0., 0. };
	int area_light_id = std::visit([&](auto& shape) {return shape.area_light_id; }, NearstObj);

	if (area_light_id != -1 && dot(Normal, -ray.Direction) > 0) // is area_light
	{
		Vector3 l_e = std::get<ParsedDiffuseAreaLight>(scene.lights[area_light_id]).radiance;
		color = l_e;
	}
	return color;
}
