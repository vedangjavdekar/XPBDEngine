#include "PositionalConstraint.h"

#include "Engine/Entity.h"
#include "Constraints/TransformationData.h"

#include "raylib.h"
#include "raymath.h"
#include "imgui.h"
#include "Engine/DebugDrawing.h"

#include "Utils/EigenToRaylib.h"

namespace Simulation
{
	void PositionalConstraint::Solve(const TransformationData& data, const float substepTime)
	{
		using namespace Eigen;
		const Vector3f deltaX = (Entity1->Position - Entity2->Position) - TargetDistance;
		Solve(data, substepTime, deltaX);
	}

	void PositionalConstraint::Solve(const TransformationData& data, float substepTime, Eigen::Vector3f error)
	{
		using namespace Eigen;

		if (!Entity1 || !Entity2)
		{
			return;
		}

		const float c = error.norm();
		// Error too small
		if (c <= FLT_EPSILON)
		{
			return;
		}
		const Vector3f n = error / c;

		const Vector3f r1CrossN = data.WorldR1.cross(n);
		const Vector3f r2CrossN = data.WorldR2.cross(n);

		const float alphaTilde = Compliance / (substepTime * substepTime);

		// Vector multiplication with a transpose results in dot product.
		const float e1InvMass = Entity1->InverseMass + r1CrossN.dot(data.Entity1InvTensor * r1CrossN);
		const float e2InvMass = Entity2->InverseMass + r2CrossN.dot(data.Entity2InvTensor * r2CrossN);
		const float invMassSum = e1InvMass + e2InvMass;
		if (invMassSum <= FLT_EPSILON)
		{
			return;
		}

		const float deltaLambda = (-c - alphaTilde * Lambda) / (invMassSum + alphaTilde);
		const Vector3f positionalImpulse = deltaLambda * n;
		if (!Entity1->IsStaticBody)
		{
			Entity1->Position += (Entity1->InverseMass * positionalImpulse);
		}

		if (!Entity2->IsStaticBody)
		{
			Entity2->Position += (-Entity2->InverseMass * positionalImpulse);
		}

		const Vector3f deltaQ1 = data.Entity1InvTensor * (data.WorldR1.cross(positionalImpulse));
		const Vector3f deltaQ2 = data.Entity2InvTensor * (data.WorldR2.cross(positionalImpulse));

		if (!Entity1->IsStaticBody && !Entity1->IsStaticForCorrection)
		{
			Entity1->Rotation.coeffs() += 0.5f * (Quaternionf(0.0f, deltaQ1(0), deltaQ1(1), deltaQ1(2)) * Entity1->Rotation).coeffs();
			Entity1->Rotation.normalize();
		}

		if (!Entity2->IsStaticBody && !Entity2->IsStaticForCorrection)
		{
			Entity2->Rotation.coeffs() += (-0.5f * (Quaternionf(0.0f, deltaQ2(0), deltaQ2(1), deltaQ2(2)) * Entity2->Rotation).coeffs());
			Entity2->Rotation.normalize();
		}


		Lambda += deltaLambda;
	}

	void PositionalConstraint::DrawConstraint()
	{
		using namespace Utils::Math;

		if (!Entity1 || !Entity2)
		{
			return;
		}

		Color constraintColor = RED;

		const Eigen::Vector3f deltaX = (Entity1->Position - Entity2->Position) - TargetDistance;
		const float errorDistSq = deltaX.squaredNorm();

		if (errorDistSq <= FLT_EPSILON)
		{
			constraintColor = GREEN;
		}

		Engine::DebugDrawing::DrawDebugLine(ToVector3(Entity1->Position), ToVector3(Entity2->Position), constraintColor, 0.0f);
	}
}