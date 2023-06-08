#pragma once
#include <vector>

#include "3rdparty/pcg.h"


struct Distribution1D
{
    std::vector<Real> pdf, cdf;
    Real funcIntegral;
    Distribution1D() = default;
    Distribution1D(std::vector<Real>& func)
    {
        pdf = func;
        cdf.resize(func.size() + 1);
        cdf.push_back(0.);
        for (int i = 0; i < (int)func.size(); i++)
        {
            cdf[i + 1] = cdf[i] + pdf[i];
        }
        funcIntegral = cdf[func.size()];
        for (int i = 0; i < (int)func.size(); i++)
        {
            cdf[i] /= funcIntegral;
        }
        cdf[func.size()] = 1.;
    }
    int sample(pcg32_state& rng)
    {
        Real random_p = next_pcg32_real<Real>(rng);
        int index = 0;
        index = (int)(std::lower_bound(cdf.begin(), cdf.end(), random_p) - cdf.begin())-1;
        return index;
    }
    Real GetPDF(int index)
    {
        return pdf[index];
    }
};

struct Distribution2D
{
    // marginal_rows: marginal p(v)  each row.
    std::shared_ptr<Distribution1D> marginal_rows;
    // conditional_v: conditional p(u|v) for each(u,v)
    std::vector<std::shared_ptr<Distribution1D>> conditional_v;
    int width, height;
    Distribution2D() = default;
    Distribution2D(const std::vector<Real>& func, int width, int height)
		:width(width), height(height)
    {
	    for(int v = 0; v < height; v++)
	    {
            std::vector<Real> row = { func.begin() + v*width, func.begin() + (v+1)*width-1 };
            conditional_v.emplace_back(new Distribution1D(row));
	    }
        std::vector<Real> marginal_func;
        for(int v = 0; v < height; v++)
        {
            marginal_func.push_back(conditional_v[v]->funcIntegral);
        }
        marginal_rows = std::make_shared<Distribution1D>(marginal_func);
    }
    Vector2i sample(pcg32_state& rng)
    {
        int row_index = marginal_rows->sample(rng);
        return { conditional_v[row_index]->sample(rng), row_index};
    }
    Real GetPDF(Vector2i uv)
    {
        int u = uv[0];
        int v = uv[1];
        return conditional_v[v]->GetPDF(u);
    }
};


