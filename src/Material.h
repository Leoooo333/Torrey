#pragma once
#include "3rdparty/stb_image.h"
#include "vector.h"
#include <variant>

#include "image.h"
#include "Texture.h"


using Color = std::variant<Vector3, Texture>;

struct Diffuse {
    Color reflectance;
};

struct Mirror {
    Color reflectance;
};

struct Plastic {
    Real eta; // index of refraction
    Color reflectance;
};

struct Phong {
    Color reflectance; // Ks
    Real exponent; // alpha
};

struct BlinnPhong {
    Color reflectance; // Ks
    Real exponent; // alpha
};

struct BlinnPhongMicrofacet {
    Color reflectance; // Ks
    Real exponent; // alpha
};


struct Glass {
    Real eta; // index of refraction
    Color reflectance;
};

struct Volume {
    Real thickness; // thickness of volume
    Color reflectance;
};

using Material = std::variant<Diffuse,
    Mirror,
    Plastic,
    Phong,
    BlinnPhong,
    BlinnPhongMicrofacet,
    Glass,
    Volume>;

