#pragma once
#include "3rdparty/stb_image.h"
#include "vector.h"
#include <variant>

#include "image.h"

struct Texture
{
    ParsedImageTexture parsedImageTexture;
    /*{
        fs::path filename;
        Real uscale = 1, vscale = 1;
        Real uoffset = 0, voffset = 0;
    };*/
    std::shared_ptr<Image3> data;
    int width = 0, height = 0;
    int bytes_per_scanline;

    Texture(ParsedImageTexture p_ImageTexture)
    {
        parsedImageTexture = p_ImageTexture;
        data = std::make_shared<Image3>(imread3(parsedImageTexture.filename.string().c_str()));

        if (data->height == 0 && data->width == 0) {
            std::cerr << "ERROR: Could not load texture image file '" << parsedImageTexture.filename.string().c_str() << "'.\n";
            width = height = 0;
        }
        width = data->width;
        height = data->height;
    }
    ~Texture()
    {
    	data.reset();
    }
    Vector3 GetColor(Real u, Real v)
    {
        if (data == nullptr)
            return Vector3(0., 1., 1.);

        Real x = width * modulo(parsedImageTexture.uscale * u + parsedImageTexture.uoffset, 1.);
        Real y = height * modulo(parsedImageTexture.vscale * v + parsedImageTexture.voffset, 1.);
        auto i = static_cast<int>(x);
        auto j = static_cast<int>(y);


        // Clamp integer mapping, since actual coordinates should be less than 1.0
        if (i >= width)  i = width - 1;
        if (j >= height) j = height - 1;

        auto pixel_00 = (*data)(i, j);
        auto pixel_01 = (*data)(i, modulo(j+1, height));
        auto pixel_10 = (*data)(modulo(i + 1, width), j);
        auto pixel_11 = (*data)(modulo(i + 1, width), modulo(j + 1, height));

        auto pixel = pixel_00* ((Real)i + 1. - x)* ((Real)j + 1. - y)
            + pixel_10 * (x - (Real)i) * ((Real)j + 1. - y)
            + pixel_01 * ((Real)i + 1. - x) * (y - (Real)j)
            + pixel_11 * (x - (Real)i) * (y - (Real)j);
        return pixel;
    }
};

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

using Material = std::variant<Diffuse,
    Mirror,
    Plastic,
    Phong,
    BlinnPhong,
    BlinnPhongMicrofacet>;

