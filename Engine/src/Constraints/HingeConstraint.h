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

		float LimitAngleMin = 0.0f;
		float LimitAngleMax = 0.0f;

		bool LimitAngle = false;

		float LambdaAlignAxis = 0.0f;
		float LambdaLimitAxis = 0.0f;
		float LambdaPositional = 0.0f;

		virtual void Init()override;

		virtual void Solve(const TransformationData& data, const float substepTime) override;

		virtual void DrawConstraint() override;
	};
}