#pragma once
#include <vector>
#include "Eigen/Dense"
#include "raylib.h"

namespace Simulation
{
	struct PhysicalForce
	{
		Eigen::Vector3f Position = Eigen::Vector3f::Zero();
		Eigen::Vector3f Vector = Eigen::Vector3f::Zero();
		bool IsLocal = false;
	};

	struct Entity
	{
		// Reset Transform
		Eigen::Vector3f ResetPosition;
		Eigen::Quaternionf ResetRotation;
		Eigen::Vector3f ResetScale;
		Eigen::Vector3f ResetAngularVelocity;
		Eigen::Vector3f ResetLinearVelocity;

		// Transform
		Eigen::Vector3f Position;
		Eigen::Quaternionf Rotation;
		Eigen::Vector3f Scale;

		// XPBD Data
		float InverseMass;
		Eigen::Matrix3f InertiaTensor;
		Eigen::Matrix3f InverseInertiaTensor;

		Eigen::Vector3f AngularVelocity;
		Eigen::Vector3f LinearVelocity;

		bool IsStaticBody;
		bool IsActive;
		bool IsStaticForCorrection;

		std::vector<PhysicalForce> Forces;

		Eigen::Vector3f PrevPosition;
		Eigen::Quaternionf PrevRotation;

		// Drawing
		bool IsParticle;
		float DrawRadius;
		Model RenderModel;
		Color RenderColor;

		Entity()
			:
			ResetPosition(Eigen::Vector3f::Zero()),
			ResetRotation(Eigen::Quaternionf::Identity()),
			ResetScale(Eigen::Vector3f::Ones()),
			ResetLinearVelocity(Eigen::Vector3f::Zero()),
			ResetAngularVelocity(Eigen::Vector3f::Zero()),
			Position(Eigen::Vector3f::Zero()),
			Rotation(Eigen::Quaternionf::Identity()),
			Scale(Eigen::Vector3f::Ones()),
			InverseMass(1.0f),
			InertiaTensor(Eigen::Matrix3f::Identity()),
			InverseInertiaTensor(Eigen::Matrix3f::Identity()),
			AngularVelocity(Eigen::Vector3f::Zero()),
			LinearVelocity(Eigen::Vector3f::Zero()),
			IsStaticBody(false),
			IsActive(true),
			IsStaticForCorrection(false),
			PrevPosition(Eigen::Vector3f::Zero()),
			PrevRotation(Eigen::Quaternionf::Identity()),
			IsParticle(false),
			DrawRadius(0.1f),
			RenderModel(),
			RenderColor(WHITE)
		{
		}

		void AddForce(Simulation::PhysicalForce force)
		{
			if (force.IsLocal)
			{
				Eigen::Matrix3f RotationMatrix = Rotation.toRotationMatrix();
				force.Vector = RotationMatrix * force.Vector;

				// Not considering the translation as we want the force to be centered at the 
				Eigen::Affine3f affine;
				affine = RotationMatrix * Eigen::Scaling(Scale);
				const Eigen::Matrix4f modelMatrix = affine.matrix();
				Eigen::Vector4f positionVector(force.Position.x(), force.Position.y(), force.Position.z(), 1.0f);

				force.Position = (modelMatrix * positionVector).head<3>();
			}

			Forces.push_back(force);
		}

		Eigen::Vector3f GetTotalForce()const
		{
			Eigen::Vector3f total = Eigen::Vector3f::Zero();
			for (const auto& force : Forces)
			{
				total += force.Vector;
			}

			return total;
		}

		Eigen::Vector3f GetTotalTorque()const
		{
			// All bodies are assumed to have origin as center of mass.
			const Eigen::Vector3f CM = Eigen::Vector3f::Zero();

			Eigen::Vector3f total = Eigen::Vector3f::Zero();
			for (const auto& force : Forces)
			{
				Eigen::Vector3f r = force.Position; // - CM; // Redundant operation
				total += r.cross(force.Vector);
			}

			return total;
		}

		void Reset() 
		{
			Position = ResetPosition;
			Rotation = ResetRotation;
			Scale = ResetScale;

			AngularVelocity = ResetAngularVelocity;
			LinearVelocity = ResetLinearVelocity;
		}
	};
}