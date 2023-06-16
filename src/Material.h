#pragma once
#include "3rdparty/stb_image.h"
#include "vector.h"
#include <variant>

#include "image.h"
#include "Texture.h"


using Color = std::variant<Vector3, Texture>;
using NormalMap = Texture;

struct Diffuse {
    Color reflectance;
    NormalMap normal_map;
};

struct Mirror {
    Color reflectance;
    NormalMap normal_map;
};

struct Plastic {
    Real eta; // index of refraction
    Color reflectance;
    NormalMap normal_map;
};

struct Phong {
    Color reflectance; // Ks
    Real exponent; // alpha
    NormalMap normal_map;
};

struct BlinnPhong {
    Color reflectance; // Ks
    Real exponent; // alpha
    NormalMap normal_map;
};

struct BlinnPhongMicrofacet {
    Color reflectance; // Ks
    Real exponent; // alpha
    NormalMap normal_map;
};


struct Glass {
    Real eta; // index of refraction
    Color reflectance;
    NormalMap normal_map;
};

struct Volume {
    Real thickness; // thickness of volume
    Color reflectance;
    NormalMap normal_map;
};

using Material = std::variant<Diffuse,
    Mirror,
    Plastic,
    Phong,
    BlinnPhong,
    BlinnPhongMicrofacet,
    Glass,
    Volume>;

