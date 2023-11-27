#pragma once
#include <string>

#include "Constraint.h"
#include "raylib.h"

namespace Simulation
{
	struct RotationalConstraint: Constraint
	{
		Entity* Entity1;
		Entity* Entity2;

		virtual void Solve(const TransformationData& data, const float substepTime) override;
		virtual void Solve(const TransformationData& data, float substepTime, Eigen::Vector3f error) override;
	};
}