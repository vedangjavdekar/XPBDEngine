#pragma once
#include <Eigen/Dense>
#include "Engine/DebugDrawing.h"

namespace Simulation
{
	struct TransformationData;

	struct Constraint
	{
		float Lambda = 0.0f;
		float Compliance = 0.0f;

		virtual void Init() { Lambda = 0.0f; }
		virtual void DrawConstraint() {}
		virtual void Solve(const TransformationData& data, const float substepTime) = 0;
		virtual void Solve(const TransformationData& data, float substepTime, Eigen::Vector3f error) {}
	};
}