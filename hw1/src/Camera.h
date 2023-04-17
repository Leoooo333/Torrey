#pragma once
#include "image.h"
#include "Ray.h"
#include "matrix.h"

#include <variant>

#include "3rdparty/pcg.h"

enum CameraType { PERSPECTIVE_CAM, ENVIRONMENT_CAM };

struct CameraParameters
{
	Vector3 center;
	Vector3 eye;
	Vector3 upvec;
	Real focal_length = 1.;
	Real fovy = 90.;
	Real aspect = 1.;
	int width;
	int height;
	int samples_per_pixel = 1;
	Real aperture = 0;
};

class AbstractCamera
{
public:
	AbstractCamera() {};
	AbstractCamera(CameraParameters cameraParameters);
	void OnResize(int width, int height);
	Ray CaculateRayDirections(int x, int y, Vector2 offset=Vector2(0.5, 0.5), pcg32_state rng=init_pcg32());
	void ReSetCamera(CameraParameters cameraParameters);
	CameraParameters m_CameraParameters;
	Matrix4x4 m_Projection;
	Matrix4x4 m_InverseProjection;
	Matrix4x4 m_View;
	Matrix4x4 m_InverseView;
};

class PerspectiveCamera : public AbstractCamera
{
public:
	PerspectiveCamera(CameraParameters cameraParameters);
	Ray CaculateRayDirections(int x, int y, Vector2 offset = Vector2(0.5, 0.5), pcg32_state rng=init_pcg32());
};

class EnvironmentCamera : public AbstractCamera
{
public:
	EnvironmentCamera(CameraParameters cameraParameters);
	Ray CaculateRayDirections(int x, int y, Vector2 offset = Vector2(0.5, 0.5), pcg32_state rng=init_pcg32());
};

using CameraUnion = std::variant<PerspectiveCamera, EnvironmentCamera>;

CameraUnion GenerateCameraByType(CameraParameters cameraParameters, CameraType c_type);

