#include "Shape.h"
#include "transform.h"



Vector4 GetNormalByHitPoint(Vector3 hitpoint, ParsedSphere& sphere)
{
	Vector4 normal_original = Vector4(hitpoint / sphere.radius, 0.);
	return normal_original;
}
Vector4 GetNormalByHitPoint(Vector3 hitpoint, Triangle& triangle)
{

	Vector3 p1 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][0]];
	Vector3 p2 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][1]];
	Vector3 p3 = triangle.mesh->positions[triangle.mesh->indices[triangle.index][2]];
	Vector3 normal = normalize(cross((p2 - p1), (p3 - p1)));
	normal = normalize(normal);
	return Vector4(normal, 0.);
}


Color ParsedColorToColor(ParsedColor& parsed_color)
{
	if(parsed_color.index() == 0) // RGB color
	{
		return Color{ std::get<Vector3>(parsed_color) };
	}
	else // Texture
	{
		return Color{ Texture(std::get<ParsedImageTexture>(parsed_color)) };
	}
}

Scene ParsedSceneToScene(ParsedScene& parsed_scene)
{
	std::vector<Shape> shapes;
	std::vector<Material> materials;
	for (ParsedShape& shape : parsed_scene.shapes)
	{
		if (shape.index() == 0) // sphere
			shapes.push_back(Shape{ std::get<ParsedSphere>(shape) });
		else // trianglemesh
		{
			ParsedTriangleMesh& mesh = std::get<ParsedTriangleMesh>(shape);
			for (int i = 0; i < mesh.indices.size(); i++)
			{
				shapes.push_back(Shape{ Triangle{&mesh, i} });
			}
		}
	}

	for (ParsedMaterial& material : parsed_scene.materials)
	{
		if (material.index() == 0) // Diffuse
		{
			ParsedDiffuse& p_d = std::get<ParsedDiffuse>(material);
			Diffuse m_d = { ParsedColorToColor(p_d.reflectance) };
			materials.push_back(Material{ m_d });
		}
		else if(material.index() == 1) // Mirror	
		{
			ParsedMirror& p_mir = std::get<ParsedMirror>(material);
			Mirror m_mir = { ParsedColorToColor(p_mir.reflectance) };
			materials.push_back(Material{ m_mir });
		}
		else if(material.index() == 2) // Plastic
		{
			ParsedPlastic& p_plas = std::get<ParsedPlastic>(material);
			Plastic m_plas = { p_plas.eta,ParsedColorToColor(p_plas.reflectance) };
			materials.push_back(Material{ m_plas });
		}
		else if(material.index() == 3) // Phong
		{
			ParsedPhong& p_pho = std::get<ParsedPhong>(material);
			Phong m_pho = {ParsedColorToColor(p_pho.reflectance),  p_pho.exponent};
			materials.push_back(Material{ m_pho });
		}
		else if(material.index() == 4) // BlinnPhong
		{
			ParsedBlinnPhong& p_blpho = std::get<ParsedBlinnPhong>(material);
			BlinnPhong m_blpho = { ParsedColorToColor(p_blpho.reflectance),  p_blpho.exponent };
			materials.push_back(Material{ m_blpho });
		}
		else if(material.index() == 5) // BlinnPhongMicrofacet
		{
			ParsedBlinnPhongMicrofacet& p_blphomic = std::get<ParsedBlinnPhongMicrofacet>(material);
			BlinnPhongMicrofacet m_blphomic = { ParsedColorToColor(p_blphomic.reflectance),  p_blphomic.exponent };
			materials.push_back(Material{ m_blphomic });
		}
	}
	return Scene{ parsed_scene.camera,
	materials,
	parsed_scene.lights,
	shapes,
	parsed_scene.background_color,
	parsed_scene.samples_per_pixel };
}

