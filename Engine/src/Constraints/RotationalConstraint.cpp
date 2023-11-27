#include "RotationalConstraint.h"

#include "Engine/Entity.h"
#include "Constraints/TransformationData.h"

#include "raylib.h"
#include "raymath.h"
#include "imgui.h"
#include "Engine/DebugDrawing.h"

#include "Utils/EigenToRaylib.h"

namespace Simulation
{
	void RotationalConstraint::Solve(const TransformationData& data, const float substepTime)
	{
		using namespace Eigen;
		const Quaternionf qFixed = Entity1->Rotation * Entity2->Rotation.inverse();
		const Eigen::Vector3f deltaQ = Vector3f(2.0f * qFixed.x(), 2.0f * qFixed.y(), 2.0f * qFixed.z());
		Solve(data, substepTime, deltaQ);
	}

	void RotationalConstraint::Solve(const TransformationData& data, float substepTime, Eigen::Vector3f error)
	{
		using namespace Eigen;
		const float theta = error.norm();
		if (theta <= FLT_EPSILON)
		{
			return;
		}

		Eigen::Vector3f n = error / theta;

		float e1InvMass = n.dot(data.Entity1InvTensor * n);
		float e2InvMass = n.dot(data.Entity2InvTensor * n);

		const float alphaTilde = Compliance / (substepTime * substepTime);

		const float invMassSum = e1InvMass + e2InvMass;

		if (invMassSum <= FLT_EPSILON)
		{
			return;
		}

		const float deltaLambda = (-theta - alphaTilde * Lambda) / (invMassSum + alphaTilde);

		const Vector3f positionalImpulse = deltaLambda * n;

		if (!Entity1->IsStaticBody && !Entity1->IsStaticForCorrection)
		{
			const Eigen::Vector3f e1IinvP = data.Entity1InvTensor * positionalImpulse;
			const Eigen::Quaternionf e1IPQuat{ 0.0f,e1IinvP.x(),e1IinvP.y(),e1IinvP.z() };
			Entity1->Rotation.coeffs() += 0.5f * (e1IPQuat * Entity1->Rotation).coeffs();
			Entity1->Rotation.normalize();
		}

		if (!Entity2->IsStaticBody && !Entity2->IsStaticForCorrection)
		{
			const Eigen::Vector3f e2IinvP = data.Entity2InvTensor * positionalImpulse;
			const Eigen::Quaternionf e2IPQuat{ 0.0f,e2IinvP.x(),e2IinvP.y(),e2IinvP.z() };
			Entity2->Rotation.coeffs() += (-0.5f * (e2IPQuat * Entity2->Rotation).coeffs());
			Entity2->Rotation.normalize();
		}
		Lambda += deltaLambda;
	}
}