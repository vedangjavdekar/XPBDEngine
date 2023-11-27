#pragma once
#if 0
#include <string>
#include "Constraint.h"
#include "raylib.h"
#include "Eigen/Dense"

namespace Simulation
{
	struct VolumeConstraint: Constraint
	{
		Eigen::Vector3f dC[4];
		Entity* Entities[4];
		float TargetVolume;

		void Solve(const float substepTime) override;
		void DrawConstraint() override;
	};
}
#endif