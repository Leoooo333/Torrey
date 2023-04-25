#pragma once
#include "vector.h"


class Variables
{
public:
	Variables(){};
public:
	CameraParameters cameraParameters =
	{
		Vector3(0., 0.,  -1.),
		Vector3(0., 0., 0.),
		Vector3(0., 1., 0.),
		1.,
		90.,
		1.,
		640,
		480
	};
	static const int maxlights = 10;
	int numlights = 0;

	static const int maxobjects = 100000;

	static const int maxverts = 100;
	int numvertexs = 0;

	int numobjects;

	Real attenuation_const = 0;
	Real attenuation_linear = 0;
	Real attenuation_quadratic = 1.;

	int max_depth = 1;
	bool isPrimary_ray = true;

	int tile_size = 32;
	int parallel_counts_bvh = 128;
	int tile_size_bvh = 1;

	Real epsilon = 1e-4;
	Real t_min = epsilon;
	Real t_max = (1. - epsilon) * 15000.;
	int motion_blur_samples = 0;

	CameraType cameraType = PERSPECTIVE_CAM;
};