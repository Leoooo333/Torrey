#include "Camera.h"

#include <3rdparty/pcg.h>

#include "vector.h"
#include "transform.h"



AbstractCamera::AbstractCamera(CameraParameters cameraParameters)
{
	ReSetCamera(cameraParameters);
}

PerspectiveCamera::PerspectiveCamera(CameraParameters cameraParameters)
{
	ReSetCamera(cameraParameters);
}

EnvironmentCamera::EnvironmentCamera(CameraParameters cameraParameters)
{
	ReSetCamera(cameraParameters);
}

Ray AbstractCamera::CaculateRayDirections(int x, int y, Vector2 offset, pcg32_state rng) { return Ray{}; }

Ray PerspectiveCamera::CaculateRayDirections(int x, int y, Vector2 offset, pcg32_state rng)
{
	Vector2 coord = { (offset.x + (Real)x) / m_CameraParameters.width, (offset.y + (Real)y) / m_CameraParameters.height };
	coord = coord * 2. - 1.;
	Real aspect_screen = (Real)m_CameraParameters.width / (Real)m_CameraParameters.height;
	Real fovx = m_CameraParameters.fovy * m_CameraParameters.aspect;
	coord.x = coord.x * tan(radians(fovx / 2)) * aspect_screen;
	coord.y = coord.y * tan(radians(m_CameraParameters.fovy / 2));
	Vector3 w = normalize(m_CameraParameters.eye - m_CameraParameters.center);
	Vector3 b = m_CameraParameters.upvec;
	Vector3 u = normalize(cross(b, w));
	Vector3 v = cross(w, u);


	Vector3 target = coord.x * u + coord.y * v - w;
	Vector3 rayOrigin = m_CameraParameters.eye;
	Vector3 rayDirection;
	if (m_CameraParameters.aperture > 0)
	{
		Real random_theta = 2 * c_PI * next_pcg32_real<double>(rng);
		Real random_radius = (m_CameraParameters.aperture / 2) * next_pcg32_real<double>(rng);
		Vector2 random_direction = {sin(random_theta), cos(random_theta)};

		//Vector3 a = target;
		//a += random_radius * random_direction.x * u +  random_radius * random_direction.y * v;
		//rayDirection = normalize(a - rayOrigin);
		rayOrigin += random_radius * random_direction.x * u +  random_radius * random_direction.y * v;
		
		rayDirection = normalize(target - rayOrigin);
	}
	else
		rayDirection = normalize(target);
	Ray ray = { rayOrigin, rayDirection };
	return ray;
}

Ray EnvironmentCamera::CaculateRayDirections(int x, int y, Vector2 offset, pcg32_state rng)
{
	Vector2 coord = { (offset.x + (Real)x) / m_CameraParameters.width, (offset.y + (Real)y) / m_CameraParameters.height };

	Real phi = 2 * c_PI * coord.x;
	Real theta = c_PI * coord.y;
	//float aspect_screen = (float)m_CameraParameters.width / (float)m_CameraParameters.height;
	//float fovx = m_CameraParameters.fovy * m_CameraParameters.aspect;
	//coord.x = coord.x * tan(radians(fovx / 2)) * aspect_screen;
	//coord.y = coord.y * tan(radians(m_CameraParameters.fovy / 2));
	//Vector4 w = Vector4(normalize(m_CameraParameters.eye - m_CameraParameters.center), 0.);
	//Vector3 b = m_CameraParameters.upvec;
	//Vector4 u = Vector4(normalize(cross(b, Vector3(w.x, w.y, w.z))), 0.);
	//Vector4 v = Vector4(cross(Vector3(w.x, w.y, w.z), Vector3(u.x, u.y, u.z)), 0.);
	//Vector4 target = u * coord.x + v * coord.y - w;
	Vector3 rayDirection = Vector3{ std::sin(theta) * std::cos(phi),
		std::cos(theta),
		std::sin(theta) * std::sin(phi) };
	Vector3 rayOrigin = m_CameraParameters.eye;
	Ray ray = { rayOrigin, rayDirection };
	return ray;
}
void AbstractCamera::ReSetCamera(CameraParameters cameraParameters)
{
	m_CameraParameters = cameraParameters;
	m_Projection = perspective(m_CameraParameters.fovy);
	m_InverseProjection = inverse(m_Projection);
	m_View = look_at(m_CameraParameters.eye, m_CameraParameters.center, m_CameraParameters.upvec);
	m_InverseView = inverse(m_View);
}
void AbstractCamera::OnResize(int width, int height)
{
	m_CameraParameters.width = width;
	m_CameraParameters.height = height;
}

CameraUnion GenerateCameraByType(CameraParameters cameraParameters, CameraType c_type)
{
	
	if (c_type == PERSPECTIVE_CAM)
	{
		PerspectiveCamera camera({ cameraParameters });
		CameraUnion cam{camera};
		return cam;
	}
	else
	{
		EnvironmentCamera camera({ cameraParameters });
		CameraUnion cam{ camera };
		return cam;
	}
}