#pragma once
#include <string>

#include "Constraint.h"
#include "raylib.h"

namespace Simulation
{
	struct Entity;

	struct PositionalConstraint: Constraint
	{
		Eigen::Vector3f LocalR1;
		Eigen::Vector3f LocalR2;
		Eigen::Vector3f TargetDistance;

		Entity* Entity1 = nullptr;
		Entity* Entity2 = nullptr;

		virtual void Solve(const TransformationData& data, const float substepTime) override;
		virtual void Solve(const TransformationData& data, const float substepTime, Eigen::Vector3f error) override;
		virtual void DrawConstraint() override;
	};
}