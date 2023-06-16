#pragma once

#include "torrey.h"
#include "vector.h"
#include <filesystem>
#include <variant>
#include <vector>

#include "Distribution.h"
#include "Texture.h"
#include "matrix.h"

struct ParsedCamera {
    Vector3 lookfrom;
    Vector3 lookat;
    Vector3 up;
    Real vfov;
    int width, height;
};


using ParsedColor = std::variant<Vector3 /* RGB */, ParsedImageTexture>;
using ParsedNormalMap = ParsedImageTexture;


struct ParsedDiffuse {
    ParsedColor reflectance;
    ParsedNormalMap normal_map;
};

struct ParsedMirror {
    ParsedColor reflectance;
    ParsedNormalMap normal_map;
};

struct ParsedPlastic {
    Real eta; // index of refraction
    ParsedColor reflectance;
    ParsedNormalMap normal_map;
};

struct ParsedGlass {
    Real eta; // index of refraction
    ParsedColor reflectance;
    ParsedNormalMap normal_map;
};

struct ParsedVolume {
    Real thickness; // thickness of volume
    ParsedColor reflectance;
    ParsedNormalMap normal_map;
};

struct ParsedPhong {
    ParsedColor reflectance; // Ks
    Real exponent; // alpha
    ParsedNormalMap normal_map;
};

struct ParsedBlinnPhong {
    ParsedColor reflectance; // Ks
    Real exponent; // alpha
    ParsedNormalMap normal_map;
};

struct ParsedBlinnPhongMicrofacet {
    ParsedColor reflectance; // Ks
    Real exponent; // alpha
    ParsedNormalMap normal_map;
};

using ParsedMaterial = std::variant<ParsedDiffuse,
                                    ParsedMirror,
                                    ParsedPlastic,
                                    ParsedPhong,
                                    ParsedBlinnPhong,
                                    ParsedBlinnPhongMicrofacet,
									ParsedGlass,
									ParsedVolume>;

struct ParsedPointLight {
    Vector3 position;
    Vector3 intensity;
};

struct ParsedDiffuseAreaLight {
    int shape_id;
    Vector3 radiance;
};

struct ParsedEnvMap
{
    Texture texture;
    Matrix4x4 light_to_world, world_to_light;
    Distribution2D dist_2d;
    Real intensity_scale = 1.;
    ParsedEnvMap(ParsedImageTexture parsed_ImageTexture, Matrix4x4 light_to_world, Real intensity_scale)
    {
        texture = Texture(parsed_ImageTexture);
        this->light_to_world = light_to_world;
        world_to_light = inverse(light_to_world);
        this->intensity_scale = intensity_scale;
        std::vector<Real> luminances;
        std::vector<Vector3>& env_image = texture.mipMap.pyramid[0]->data;
        for (int v = 0; v < texture.height; v++)
        {
            for (int u = 0; u < texture.width; u++)
            {
                Real sinTheata = sin(c_PI * (Real)v / (Real)texture.height);
                Vector3& pixel = env_image[v * texture.width + u];
                Real luminance = pixel[0] * Real(0.212671) + pixel[1] * Real(0.715160) + pixel[2] * Real(0.072169);
                luminances.push_back(luminance * sinTheata);
            }
        }
        dist_2d = Distribution2D(luminances, texture.width, texture.height);
    }
    Vector2 GetUV(Vector3 direction)
    {
        return{ (atan2(-direction.z, direction.x) + c_PI) / (2. * c_PI),
                acos(direction.y) / c_PI };
    }
};
using ParsedLight = std::variant<ParsedPointLight, ParsedDiffuseAreaLight, ParsedEnvMap>;

/// A Shape is a geometric entity that describes a surface. E.g., a sphere, a triangle mesh, a NURBS, etc.
/// For each shape, we also store an integer "material ID" that points to a material, and an integer
/// "area light ID" that points to a light source if the shape is an area light. area_lightID is set to -1
/// if the shape is not an area light.
struct ParsedShapeBase {
    int material_id = -1;
    int area_light_id = -1;
};

struct ParsedSphere : public ParsedShapeBase {
    Vector3 position;
    Real radius;
};

struct ParsedTriangleMesh : public ParsedShapeBase {
    std::vector<Vector3> positions;
    std::vector<Vector3i> indices;
    std::vector<Vector3> normals;
    std::vector<Vector2> uvs;
};

using ParsedShape = std::variant<ParsedSphere, ParsedTriangleMesh>;

inline void set_material_id(ParsedShape &shape, int material_id) {
    std::visit([&](auto &s) { s.material_id = material_id; }, shape);
}
inline void set_area_light_id(ParsedShape &shape, int area_light_id) {
    std::visit([&](auto &s) { s.area_light_id = area_light_id; }, shape);
}
inline int get_material_id(const ParsedShape &shape) {
    return std::visit([&](const auto &s) { return s.material_id; }, shape);
}
inline int get_area_light_id(const ParsedShape &shape) {
    return std::visit([&](const auto &s) { return s.area_light_id; }, shape);
}
inline bool is_light(const ParsedShape &shape) {
    return get_area_light_id(shape) >= 0;
}

struct ParsedScene {
    ParsedCamera camera;
    std::vector<ParsedMaterial> materials;
    std::vector<ParsedLight> lights;
    std::vector<ParsedShape> shapes;
    Vector3 background_color;
    int samples_per_pixel;
};

ParsedScene parse_scene(const fs::path &filename);
