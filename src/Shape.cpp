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



Scene ParsedSceneToScene(ParsedScene& parsed_scene)
{
	std::vector<Shape> shapes;
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
	return Scene{ parsed_scene.camera,
	parsed_scene.materials,
	parsed_scene.lights,
	shapes,
	parsed_scene.background_color,
	parsed_scene.samples_per_pixel };
}

