#include "HingeConstraint.h"

#include "PositionalConstraint.h"
#include "RotationalConstraint.h"

#include "Engine/Entity.h"
#include "Constraints/TransformationData.h"

#include "raylib.h"
#include "raymath.h"
#include "imgui.h"
#include "Engine/DebugDrawing.h"

#include "Utils/EigenToRaylib.h"

#include <iostream>

namespace Simulation
{
	static RotationalConstraint ConstructAlignmentConstraint(const HingeConstraint& constraint)
	{
		RotationalConstraint rot;
		rot.Compliance = constraint.Compliance;
		rot.Entity1 = constraint.Entity1;
		rot.Entity2 = constraint.Entity2;
		rot.Lambda = constraint.LambdaAlignAxis;
		return rot;
	}

	static PositionalConstraint ConstructPositionalConstraint(const HingeConstraint& constraint)
	{
		PositionalConstraint pos;
		pos.Compliance = 0.0f;
		pos.Entity1 = constraint.Entity1;
		pos.Entity2 = constraint.Entity2;
		pos.Lambda = constraint.LambdaPositional;
		pos.LocalR1 = constraint.E1AttachPoint;
		pos.LocalR2 = constraint.E2AttachPoint;
		pos.TargetDistance = Eigen::Vector3f::Zero();
		return pos;
	}


	void HingeConstraint::Init()
	{
		LambdaAlignAxis = 0.0f;
		LambdaLimitAxis = 0.0f;
		LambdaPositional = 0.0f;
	}

	void HingeConstraint::Solve(const TransformationData& data, const float substepTime)
	{
		using namespace Eigen;

		if (!Entity1 || !Entity2)
		{
			return;
		}

		const Eigen::Vector3f e1AlignAxisWorld = Entity1->Rotation.toRotationMatrix() * E1AlignAxis;
		const Eigen::Vector3f e2AlignAxisWorld = Entity2->Rotation.toRotationMatrix() * E2AlignAxis;
		const Eigen::Vector3f deltaQ = e1AlignAxisWorld.cross(e2AlignAxisWorld);

		RotationalConstraint alignmentConstraint = ConstructAlignmentConstraint(*this);
		alignmentConstraint.Solve(data, substepTime, deltaQ);
		LambdaAlignAxis = alignmentConstraint.Lambda;


		const Eigen::Vector3f p1 = Entity1->Position + data.WorldR1;
		const Eigen::Vector3f p2 = Entity2->Position + data.WorldR2;
		const Eigen::Vector3f deltaX = p1 - p2;
		
		PositionalConstraint positionalConstraint = ConstructPositionalConstraint(*this);
		positionalConstraint.Solve(data, substepTime, deltaX);
		LambdaPositional += positionalConstraint.Lambda;
	}

	void HingeConstraint::DrawConstraint()
	{
	}
}