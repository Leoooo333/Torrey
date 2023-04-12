#pragma once
#include "vector.h"


class Variables
{
public:
	Variables(){};
public:
	static const int maxlights = 10;
	int numlights = 0;

	static const int maxobjects = 100000;

	static const int maxverts = 100;
	int numvertexs = 0;

	int numobjects;

	float attenuation_const = 1;
	float attenuation_linear = 0;
	float attenuation_quadratic = 0;

	int maxdepth = 0;
	int maxdepth_refract = 0;
	bool isPrimary_ray = true;

	CameraType cameraType = PERSPECTIVE_CAM;
};