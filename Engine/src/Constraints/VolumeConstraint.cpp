#if 0
#include "VolumeConstraint.h"
#include "Engine/Entity.h"

#include "raylib.h"
#include "raymath.h"
#include "imgui.h"

namespace Simulation
{
	VolumeConstraint::VolumeConstraint(Entity* p1, Entity* p2, Entity* p3, Entity* p4, float targetVolume /*= 0.0f*/)
		:Entities{ p1, p2, p3, p4 }
		, dC{
			Eigen::Vector3f::Zero(),
			Eigen::Vector3f::Zero(),
			Eigen::Vector3f::Zero(),
			Eigen::Vector3f::Zero()
		}
		, TargetVolume(targetVolume)
	{
		if (TargetVolume == 0.0f)
		{
			const Eigen::Vector3f x2_minus_x1 = Entities[1]->Position - Entities[0]->Position;
			const Eigen::Vector3f x3_minus_x1 = Entities[2]->Position - Entities[0]->Position;
			const Eigen::Vector3f x4_minus_x1 = Entities[3]->Position - Entities[0]->Position;
			const Eigen::Vector3f x3_minus_x2 = Entities[2]->Position - Entities[1]->Position;
			const Eigen::Vector3f x4_minus_x2 = Entities[3]->Position - Entities[1]->Position;

			TargetVolume = (x2_minus_x1.cross(x3_minus_x1)).dot(x4_minus_x1) / 6.0f;
		}
	}

	void VolumeConstraint::Solve(const float substepTime)
	{
		float sumInverseMass = 0.0f;
		for (auto& particle : Entities)
		{
			sumInverseMass += particle->InverseMass;
		}
		if (sumInverseMass == 0)
		{
			return;
		}

		const Eigen::Vector3f x2_minus_x1 = Entities[1]->Position - Entities[0]->Position;
		const Eigen::Vector3f x3_minus_x1 = Entities[2]->Position - Entities[0]->Position;
		const Eigen::Vector3f x4_minus_x1 = Entities[3]->Position - Entities[0]->Position;
		const Eigen::Vector3f x3_minus_x2 = Entities[2]->Position - Entities[1]->Position;
		const Eigen::Vector3f x4_minus_x2 = Entities[3]->Position - Entities[1]->Position;

		const float currentVolume = (x2_minus_x1.cross(x3_minus_x1)).dot(x4_minus_x1);
		const float deltaVolume = currentVolume - 6 * TargetVolume;

		dC[0] = x4_minus_x2.cross(x3_minus_x2);
		dC[1] = x3_minus_x1.cross(x4_minus_x1);
		dC[2] = x4_minus_x1.cross(x2_minus_x1);
		dC[3] = x2_minus_x1.cross(x3_minus_x1);

		const float alpha = Compliance / (substepTime * substepTime);
		float denom = 0.0f;
		for (int i = 0; i < 4; ++i)
		{
			denom += Entities[i]->InverseMass * (dC[i].dot(dC[i]));
		}

		denom += alpha;

		if (denom == 0.0f)
		{
			denom = FLT_EPSILON;
		}

		const float deltaLamba = -(deltaVolume - alpha * Lambda) / denom;
		for (int i = 0; i < 4; ++i)
		{
			Entities[i]->Position += (Entities[i]->InverseMass * deltaLamba) * dC[i];
		}
		Lambda += deltaLamba;
	}

	void VolumeConstraint::DrawConstraint()
	{

	}
}
#endif