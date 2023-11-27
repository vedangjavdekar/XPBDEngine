#include "ParticlesScene.h"
#include "Engine/Application.h"
#include "raymath.h"
#include "imgui.h"

#include "Engine/Entity.h"
#include "Engine/DebugDrawing.h"
#include "Constraints/PositionalConstraint.h"
#include "Constraints/TransformationData.h"

#include "Utils/EigenToRaylib.h"
#include "Utils/PhysicsUtils.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Scenes
{
	namespace
	{
		static std::array<Simulation::Entity, 5> Entities;

		static std::array<Simulation::PositionalConstraint, 3> Constraints;
		static std::array<Simulation::TransformationData, 3> TransformationData;

		static std::array<Utils::Physics::ForceInput, 2> ForceInputs;
	}

	ParticlesScene::ParticlesScene(const std::string &sceneName)
		: Scene(sceneName)
	{
		SetupEntities();
		SetupConstraints();
		SetupInputs();
	}

	void ParticlesScene::OnInit()
	{
		for (auto &particle : Entities)
		{
			particle.Reset();
		}
	}

	void ParticlesScene::OnUpdate(const float substepTime)
	{
	}

	void ParticlesScene::OnStartSimulationFrame()
	{
		if (m_EnableGravity)
		{
			for (auto &particle : Entities)
			{
				particle.AddForce(
					Simulation::PhysicalForce{
						Eigen::Vector3f::Zero(),
						Eigen::Vector3f(0.0f, m_Gravity * 1.0f / particle.InverseMass, 0.0f),
						false});
			}
		}

		for (auto &force : ForceInputs)
		{
			force.Apply();
		}
	}

	void ParticlesScene::OnUpdatePosition(const float substepTime)
	{
		for (auto &particle : Entities)
		{
			particle.PrevPosition = particle.Position;

			if (particle.IsStaticBody)
			{
				continue;
			}

			const Eigen::Vector3f totalForce = particle.GetTotalForce();
			particle.LinearVelocity += (particle.InverseMass * substepTime) * totalForce;
			particle.Position += substepTime * particle.LinearVelocity;

			Engine::DebugDrawing::DrawForceMarker(BLUE, particle.Position, particle.Rotation, particle.Position, totalForce, false, -1.0f, 0.0f);
		}
	}

	void ParticlesScene::OnSolveConstraints(const float substepTime)
	{
		for (int i = 0; i < GetNumPosIterations(); ++i)
		{
			for (size_t j = 0; j < Constraints.size(); ++j)
			{
				if (i == 0)
				{
					Constraints[j].Init();
					TransformationData[j] = GetTransformationData(Constraints[j].Entity1, Constraints[j].Entity2);
					
				}
				ComputePositionalData(TransformationData[j], Constraints[j].LocalR1, Constraints[j].LocalR2);

				Constraints[j].Solve(TransformationData[j], substepTime);
			}
		}

		if (m_GroundCollisions)
		{
			for (auto &particle : Entities)
			{
				particle.Position.y() = std::max(particle.Position.y(), 0.0f);
			}
		}
	}

	void ParticlesScene::OnPostSolveConstraints(const float substepTime)
	{
		for (auto &particle : Entities)
		{
			particle.LinearVelocity = (particle.Position - particle.PrevPosition) / substepTime;
		}
	}

	void ParticlesScene::OnEndSimulationFrame()
	{
		for (auto &particle : Entities)
		{
			particle.Forces.clear();
		}
	}

	void ParticlesScene::OnDraw()
	{
		using namespace Utils::Math;

		for (auto &constraint : Constraints)
		{
			constraint.DrawConstraint();
		}

		for (auto &particle : Entities)
		{
			DrawSphere(ToVector3(particle.Position), particle.DrawRadius, particle.RenderColor);
		}

		for (auto &forceInput : ForceInputs)
		{
			forceInput.Draw();
		}
	}

	void ParticlesScene::OnDrawEditor()
	{
		Scene::OnDrawEditor();

		if (ImGui::DragFloat("Draw Size", &m_ParticleDrawRadius, 0.05f))
		{
			for (auto &particle : Entities)
			{
				particle.DrawRadius = m_ParticleDrawRadius;
			}
			m_IsDirty = true;
		}

		ImGui::Checkbox("Enable Gravity", &m_EnableGravity);
		if (m_EnableGravity)
		{
			ImGui::DragFloat("Gravity", &m_Gravity);
		}
		ImGui::Checkbox("Ground Collisions", &m_GroundCollisions);

		ImGui::SeparatorText("Particles");

		for (int i = 0; i < Entities.size(); ++i)
		{
			std::string particleName = "Particle " + std::to_string(i);
			DrawParticle(particleName, Entities[i], i);
		}

		ImGui::SeparatorText("Force Inputs");

		for (int i = 0; i < ForceInputs.size(); ++i)
		{
			m_IsDirty |= ForceInputs[i].DrawSettings();
		}

		ImGui::SeparatorText("Constraints");
		{
			static float allCompliance = 0.0f;

			if (ImGui::InputFloat("AllCompliance", &allCompliance))
			{
				allCompliance = std::max(allCompliance, 0.0f);
				for (auto &constraint : Constraints)
				{
					constraint.Compliance = allCompliance;
				}
			}
		}
	}

	void ParticlesScene::OnShutdown()
	{
	}

	void ParticlesScene::SetupEntities()
	{
		using namespace Eigen;

		Entities[0].ResetPosition = Vector3f{-4.0f, 3.0f, 0.0f};
		Entities[1].ResetPosition = Vector3f{-3.0f, 5.0f, -1.0f};
		Entities[2].ResetPosition = Vector3f{-2.0f, 4.0f, 1.0f};

		Entities[3].ResetPosition = Vector3f{2.0f, 5.0f, 0.0f};
		Entities[4].ResetPosition = Vector3f{2.0f, 3.0f, 0.0f};
	}

	void ParticlesScene::SetupConstraints()
	{
		using namespace Eigen;

		Constraints[0].Entity1 = &Entities[0];
		Constraints[0].Entity2 = &Entities[1];
		Constraints[0].TargetDistance = Constraints[0].Entity1->ResetPosition - Constraints[0].Entity2->ResetPosition;

		Constraints[1].Entity1 = &Entities[1];
		Constraints[1].Entity2 = &Entities[2];
		Constraints[1].TargetDistance = Constraints[1].Entity1->ResetPosition - Constraints[1].Entity2->ResetPosition;

		Constraints[2].Entity1 = &Entities[3];
		Constraints[2].Entity2 = &Entities[4];
		Constraints[2].TargetDistance = Constraints[2].Entity1->ResetPosition - Constraints[2].Entity2->ResetPosition;
	}

	void ParticlesScene::SetupInputs()
	{
		ForceInputs[0].ActivationKey = KEY_K;
		ForceInputs[0].Entity = &Entities[3];

		ForceInputs[1].ActivationKey = KEY_L;
		ForceInputs[1].Entity = &Entities[4];
	}

	void ParticlesScene::DrawParticle(const std::string &particleName, Simulation::Entity &particle, size_t StoredForceIndex)
	{
		if (ImGui::TreeNode(particleName.c_str()))
		{
			m_IsDirty |= ImGui::Checkbox("Is Static", &particle.IsStaticBody);
			ImGui::DragFloat("Inverse Mass", &particle.InverseMass, 0.1f, 0.0f, 0.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp);

			ImGui::TreePop();
		}
	}
}