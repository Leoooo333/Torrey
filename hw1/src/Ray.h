#pragma once
#include "vector.h"

struct Ray
{
	Vector3 Origin;
	Vector3 Direction;
	int NearstObject;
	Real t_nearst;
};
