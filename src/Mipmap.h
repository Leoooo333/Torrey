#pragma once
#include "image.h"
const int MAX_MIP_DEPTH = 8;

template <typename T>
struct MipMap {
    std::vector<std::shared_ptr<Image<T>>> pyramid;
    MipMap() = default;
    MipMap(Image<T>& img)
    {
        pyramid.push_back(std::make_shared<Image<T>>(img));
        int max_lavel = min((int)log2(Real(max(img.width, img.height))) + 1, MAX_MIP_DEPTH);
        for (int i = 0; i < max_lavel; i++) 
        {
        	Image<T>& last_img = *pyramid[pyramid.size()-1];
            int new_w = max(last_img.width / 2, 1);
            int new_h = max(last_img.height / 2, 1);
            Image<T> new_img(new_w, new_h);
            for (int y = 0; y < new_img.height; y++) 
            {
                for (int x = 0; x < new_img.width; x++) 
                {
                	new_img(x, y) = (last_img(2 * x, 2 * y) +
									last_img(2 * x + 1, 2 * y) +
									last_img(2 * x + 1, 2 * y + 1) +
									last_img(2 * x, 2 * y + 1)) / Real(4);
                }
            }
            pyramid.push_back(std::make_shared<Image<T>>(new_img));
        }
    }
    ~MipMap()
    {
        pyramid.clear();
    }
    //bilinear interpolation, for integer level
	T Lookup(Real u, Real v, Real uscale, Real vscale, Real uoffset, Real voffset, int level)
    {
        Image<T>& current_img = *pyramid[level];

        Real x = current_img.width * modulo(uscale * u + uoffset, 1.);
        Real y = current_img.height * modulo(vscale * v + voffset, 1.);
        auto i = static_cast<int>(x);
        auto j = static_cast<int>(y);


        // Clamp integer mapping, since actual coordinates should be less than 1.0
        if (i >= current_img.width)  i = current_img.width - 1;
        if (j >= current_img.height) j = current_img.height - 1;

        auto pixel_00 = current_img(i, j);
        auto pixel_01 = current_img(i, modulo(j + 1, current_img.height));
        auto pixel_10 = current_img(modulo(i + 1, current_img.width), j);
        auto pixel_11 = current_img(modulo(i + 1, current_img.width), modulo(j + 1, current_img.height));

        auto pixel = pixel_00 * ((Real)i + 1. - x) * ((Real)j + 1. - y)
            + pixel_10 * (x - (Real)i) * ((Real)j + 1. - y)
            + pixel_01 * ((Real)i + 1. - x) * (y - (Real)j)
            + pixel_11 * (x - (Real)i) * (y - (Real)j);
        return pixel;
    }
    //trilinear interpolation, for real level
	T Lookup(Real u, Real v, Real uscale, Real vscale, Real uoffset, Real voffset, Real level)
    {
        if (level <= 0) 
        {
            return Lookup(u, v, uscale, vscale, uoffset, voffset, 0);
        }
        else if (level < Real(pyramid.size() - 1)) 
        {
            int low_level = max((int)level, 0);
            int high_level = min(low_level + 1, (int)pyramid.size() - 1);
            Real portion = level - low_level;
            return Lookup(u, v, uscale, vscale, uoffset, voffset, low_level) * (1 - portion) +
                Lookup(u, v, uscale, vscale, uoffset, voffset, high_level) * portion;
        }
        else 
        {
            return Lookup(u, v, uscale, vscale, uoffset, voffset, (int)pyramid.size() - 1);
        }
    }
};