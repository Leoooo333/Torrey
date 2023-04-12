#pragma once
#include "image.h"
#include "Ray.h"
#include "matrix.h"

#include <variant>

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
};

class AbstractCamera
{
public:
	AbstractCamera() {};
	AbstractCamera(CameraParameters cameraParameters);
	void OnResize(int width, int height);
	void CaculateRayDirections();
	void ReSetCamera(CameraParameters cameraParameters);
	CameraParameters m_CameraParameters;
	std::vector<Ray> m_Rays;
	Matrix4x4 m_Projection;
	Matrix4x4 m_InverseProjection;
	Matrix4x4 m_View;
	Matrix4x4 m_InverseView;
};

class PerspectiveCamera : public AbstractCamera
{
public:
	PerspectiveCamera(CameraParameters cameraParameters);
	void CaculateRayDirections();
};

class EnvironmentCamera : public AbstractCamera
{
public:
	EnvironmentCamera(CameraParameters cameraParameters);
	void CaculateRayDirections();
};

using CameraUnion = std::variant<PerspectiveCamera, EnvironmentCamera>;

CameraUnion GenerateCameraByType(CameraParameters cameraParameters, CameraType c_type);

