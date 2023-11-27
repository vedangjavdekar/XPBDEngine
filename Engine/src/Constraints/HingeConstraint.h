#pragma once
#include <string>

#include "Constraint.h"
#include "raylib.h"

namespace Simulation
{
	struct HingeConstraint: Constraint
	{
		Entity* Entity1 = nullptr;
		Entity* Entity2 = nullptr;

		Eigen::Vector3f E1AlignAxis;
		Eigen::Vector3f E2AlignAxis;

		Eigen::Vector3f E1LimitAxis;
		Eigen::Vector3f E2LimitAxis;

		Eigen::Vector3f E1AttachPoint; // r1
		Eigen::Vector3f E2AttachPoint; // r2

		bool LimitAngle;

		float LambdaAlignAxis;
		float LambdaLimitAxis;
		float LambdaPositional;

		virtual void Init()override;

		virtual void Solve(const TransformationData& data, const float substepTime) override;

		virtual void DrawConstraint() override;
	};
}