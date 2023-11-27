#pragma once
#include "Eigen/Dense"
#include "raylib.h"

namespace Simulation
{
	struct Particle
	{
		Eigen::Vector3f Position;
		Eigen::Vector3f PrevPosition;
		Eigen::Vector3f Velocity;
		Eigen::Vector3f Force;
		float InverseMass = 0.0f;

		float DrawRadius = 0.1f;
		Color DrawColor = WHITE;
	};
}