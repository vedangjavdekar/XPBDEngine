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

	static void LimitHingeAngle(const TransformationData& data, const float substepTime, HingeConstraint& hingeConstraint)
	{
		const float minAngleRad = hingeConstraint.LimitAngleMin /** DEG2RAD*/;
		const float maxAngleRad = hingeConstraint.LimitAngleMax /** DEG2RAD*/;
		Eigen::Vector3f e1LimitAxisWorld = hingeConstraint.Entity1->Rotation.toRotationMatrix() * hingeConstraint.E1LimitAxis;
		Eigen::Vector3f e2LimitAxisWorld = hingeConstraint.Entity2->Rotation.toRotationMatrix() * hingeConstraint.E2LimitAxis;
		Eigen::Vector3f e1AlignAxisWorld = hingeConstraint.Entity1->Rotation.toRotationMatrix() * hingeConstraint.E1AlignAxis;

		float phi = std::asin((e1LimitAxisWorld.cross(e2LimitAxisWorld)).dot(e1AlignAxisWorld));
		if (e1LimitAxisWorld.dot(e2LimitAxisWorld) < 0.0f)
		{
			phi = PI - phi;
		}

		if (phi > PI)
		{
			phi = phi - 2 * PI;
		}

		if (phi < -PI)
		{
			phi = phi + 2 * PI;
		}

		if (phi < minAngleRad || phi > maxAngleRad)
		{
			phi = Clamp(phi, minAngleRad,maxAngleRad);
			e1LimitAxisWorld = Eigen::AngleAxisf(phi, e1AlignAxisWorld) * e1LimitAxisWorld;
			Eigen::Vector3f deltaQ = e1LimitAxisWorld.cross(e2LimitAxisWorld);
			RotationalConstraint rot = ConstructAlignmentConstraint(hingeConstraint);
			rot.Solve(data, substepTime, deltaQ);
			hingeConstraint.LambdaLimitAxis += rot.Lambda;
		}
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

		if (LimitAngle)
		{
			LimitHingeAngle(data, substepTime, *this);
		}
	}

	void HingeConstraint::DrawConstraint()
	{
		using namespace Utils::Math;
		if (!Entity1 || !Entity2)
		{
			return;
		}

		Eigen::Vector3f AlignAxis1World = Entity1->Rotation.toRotationMatrix() * E1AlignAxis.normalized();
		Eigen::Vector3f AlignAxis2World = Entity2->Rotation.toRotationMatrix() * E2AlignAxis.normalized();
		Eigen::Vector3f LimitAxis1World = Entity1->Rotation.toRotationMatrix() * E1LimitAxis.normalized();
		Eigen::Vector3f LimitAxis2World = Entity2->Rotation.toRotationMatrix() * E2LimitAxis.normalized();

		DrawLine3D(ToVector3(Entity1->Position), ToVector3(Entity1->Position + 5 * AlignAxis1World), GREEN);
		DrawLine3D(ToVector3(Entity2->Position), ToVector3(Entity2->Position + 5 * AlignAxis2World), GREEN);

		DrawLine3D(ToVector3(Entity1->Position), ToVector3(Entity1->Position + 5 * LimitAxis1World), RED);
		DrawLine3D(ToVector3(Entity2->Position), ToVector3(Entity2->Position + 5 * LimitAxis2World), RED);
	}
}