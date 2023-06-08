#pragma once
#include "torrey.h"
#include "Texture.h"
#include "3rdparty/stb_image.h"
#include "image.h"
#include "Mipmap.h"
#include <map>
#include <variant>



struct ParsedImageTexture {
    fs::path filename;
    Real uscale = 1, vscale = 1;
    Real uoffset = 0, voffset = 0;
};
struct Texture
{
    ParsedImageTexture parsedImageTexture;
    MipMap<Vector3> mipMap;
    int width = 0, height = 0;
    int bytes_per_scanline;
    Texture() = default;
    Texture(ParsedImageTexture p_ImageTexture)
    {
        parsedImageTexture = p_ImageTexture;
        mipMap = MipMap(imread3(parsedImageTexture.filename.string().c_str()));

        if (mipMap.pyramid[0]->height == 0 && mipMap.pyramid[0]->width == 0) {
            std::cerr << "ERROR: Could not load texture image file '" << parsedImageTexture.filename.string().c_str() << "'.\n";
            width = height = 0;
        }
        width = mipMap.pyramid[0]->width;
        height = mipMap.pyramid[0]->height;
    }
    Vector3 GetColor(Real u, Real v)
    {
        return GetColor(u, v, 0);
    }
    Vector3 GetColor(Real u, Real v, Real level)
    {
        return mipMap.Lookup(u, v, parsedImageTexture.uscale, parsedImageTexture.vscale,
            parsedImageTexture.uoffset, parsedImageTexture.voffset, level);
    }
    Vector3 GetColor(Real u, Real v, int level)
    {
        return mipMap.Lookup(u, v, parsedImageTexture.uscale, parsedImageTexture.vscale,
            parsedImageTexture.uoffset, parsedImageTexture.voffset, (int)level);
    }
    Real GetLevel(Real footprint)
    {
        Real scaled_footprint = footprint * max(width, height) *
            max(parsedImageTexture.uscale, parsedImageTexture.vscale);
        Real level = log2(max(scaled_footprint, (Real)1e-9f));
        return level;
    }
};