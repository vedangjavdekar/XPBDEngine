#include "CubePositionalScene.h"
#include "Engine/Application.h"
#include "Constraints/PositionalConstraint.h"
#include "Constraints/TransformationData.h"
#include "raymath.h"
#include "imgui.h"

#include "Utils/EigenToRaylib.h"
#include "Utils/PhysicsUtils.h"

#include <iostream>

namespace Scenes
{
	namespace
	{
		static std::array<Simulation::Entity, 2> Entities;

		static Simulation::PositionalConstraint PositionalConstraint;
		static Simulation::TransformationData TransformationData;

		static std::array<Utils::Physics::ForceInput, 2> ForceInputs;

		static Eigen::DiagonalMatrix<float, 3> ComputeInertiaTensorForCube(float W, float H, float L)
		{
			const float volume_12 = W * H * L / 12.0f;
			const float Ixx = H * H + L * L;
			const float Iyy = W * W + L * L;
			const float Izz = W * W + H * H;

			Eigen::DiagonalMatrix<float, 3> mat(Ixx, Iyy, Izz);
			return volume_12 * mat;
		}
	}

	CubePositionalScene::CubePositionalScene(const std::string& sceneName)
		: Scene(sceneName)
	{
		SetupEntites();
		SetupConstraints();
		SetupInputs();
	}

	CubePositionalScene::~CubePositionalScene()
	{
	}

	void CubePositionalScene::OnInit()
	{
		for (auto& entity : Entities)
		{
			entity.Reset();
		}
	}

	void CubePositionalScene::OnUpdate(const float substepTime)
	{
	}

	void CubePositionalScene::OnStartSimulationFrame()
	{
		for (auto& entity : Entities)
		{
			if (m_EnableGravity)
			{
				entity.AddForce(Simulation::PhysicalForce{
					Eigen::Vector3f::Zero(),
					Eigen::Vector3f(0.0f, m_Gravity * 1.0f / entity.InverseMass, 0.0f) });
			}
		}

		for (auto& forceInput : ForceInputs)
		{
			forceInput.Apply();
		}
	}

	void CubePositionalScene::OnUpdatePosition(const float substepTime)
	{
		for (auto& entity : Entities)
		{
			// Store to Previous
			entity.PrevPosition = entity.Position;
			// Store to Previous
			entity.PrevRotation = entity.Rotation;

			if (entity.IsStaticBody)
			{
				continue;
			}

			// LINEAR MOTION
			// Velocity Update
			const Eigen::Vector3f totalForce = entity.GetTotalForce();
			entity.LinearVelocity += substepTime * entity.InverseMass * totalForce;
			// Position Update
			entity.Position += substepTime * entity.LinearVelocity;

			// ANGULAR MOTION
			// Velocity Update
			const Eigen::Vector3f totalTorque = entity.GetTotalTorque();
			entity.AngularVelocity += substepTime * entity.InverseInertiaTensor * (totalTorque - entity.AngularVelocity.cross(entity.InertiaTensor * entity.AngularVelocity));

			// Rotation Update
			const Eigen::Quaternion OmegaQuaternion = Eigen::Quaternion{
				0.0f,
				entity.AngularVelocity(0),
				entity.AngularVelocity(1),
				entity.AngularVelocity(2) };

			Eigen::Quaternionf wq = OmegaQuaternion * entity.Rotation;
			wq.coeffs() = entity.Rotation.coeffs() + substepTime * 0.5f * wq.coeffs();

			entity.Rotation = wq.normalized();

			Engine::DebugDrawing::DrawForceMarker(BLUE, entity.Position, entity.Rotation, entity.Position, totalForce, false, -1.0f, 0.0f);
		}
	}

	void CubePositionalScene::OnSolveConstraints(const float substepTime)
	{
		for (int i = 0; i < GetNumPosIterations(); ++i)
		{
			if (i == 0)
			{
				PositionalConstraint.Init();
				TransformationData = GetTransformationData(PositionalConstraint.Entity1, PositionalConstraint.Entity2);
			}

			ComputePositionalData(TransformationData, PositionalConstraint.LocalR1, PositionalConstraint.LocalR2);
			PositionalConstraint.Solve(TransformationData, substepTime);
		}
	}

	void CubePositionalScene::OnPostSolveConstraints(const float substepTime)
	{
		for (auto& entity : Entities)
		{
			entity.LinearVelocity = (entity.Position - entity.PrevPosition) / substepTime;
			const Eigen::Quaternionf deltaQ = entity.Rotation * entity.PrevRotation.inverse();
			if (deltaQ.w() > 0)
			{
				entity.AngularVelocity = (2.0f / substepTime) * Eigen::Vector3f(deltaQ.x(), deltaQ.y(), deltaQ.z());
			}
			else
			{
				entity.AngularVelocity = (-2.0f / substepTime) * Eigen::Vector3f(deltaQ.x(), deltaQ.y(), deltaQ.z());
			}
		}
	}

	void CubePositionalScene::OnEndSimulationFrame()
	{
		for (auto& entity : Entities)
		{
			entity.Forces.clear();
		}
	}

	void CubePositionalScene::OnShutdown()
	{
	}

	void CubePositionalScene::SetupEntites()
	{
		Entities[0].RenderModel = Engine::Application::Get().GetResources().CubeModel;
		Entities[1].RenderModel = Engine::Application::Get().GetResources().CubeModel;

		Entities[0].IsStaticBody = true;
		Entities[0].ResetScale = 0.25f * Eigen::Vector3f::Ones();
		Entities[0].ResetPosition = Eigen::Vector3f(0.0f, 4.0f, 0.0f);

		Entities[0].InverseMass = 1.0f;
		Entities[0].InertiaTensor = ComputeInertiaTensorForCube(0.1f, 0.1f, 0.1f);
		Entities[0].InverseInertiaTensor = Entities[0].InertiaTensor.inverse();

		Entities[1].ResetPosition = Eigen::Vector3f(0.0f, 2.0f, 0.0f);

		Entities[1].InverseMass = 1.0f;
		Entities[1].InertiaTensor = ComputeInertiaTensorForCube(1.0f, 1.0f, 1.0f);
		Entities[1].InverseInertiaTensor = Entities[1].InertiaTensor.inverse();
	}

	void CubePositionalScene::SetupConstraints()
	{
		PositionalConstraint.Entity1 = &Entities[1];
		PositionalConstraint.LocalR1 = Eigen::Vector3f(0.25f, 0.5f, 0.0f);
		PositionalConstraint.Entity2 = &Entities[0];
		PositionalConstraint.Compliance = 0.000f;
		PositionalConstraint.TargetDistance = Eigen::Vector3f(0.0f, -2.0f, 0.0f);
	}

	void CubePositionalScene::SetupInputs()
	{
		ForceInputs[0].Entity = &Entities[1];
		ForceInputs[0].ActivationKey = KEY_L;
		ForceInputs[0].ForceVector = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
		ForceInputs[0].ForcePosition = Eigen::Vector3f(0.5f, 0.0f, 0.0f);
		ForceInputs[0].IsLocal = true;

		ForceInputs[1].Entity = &Entities[1];
		ForceInputs[1].ActivationKey = KEY_K;
		ForceInputs[1].ForceVector = Eigen::Vector3f(0.0, 1.0f, 0.0f);
		ForceInputs[1].ForcePosition = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
		ForceInputs[1].IsLocal = true;
		ForceInputs[1].IsRotationalForce = true;
	}

	void CubePositionalScene::OnDraw()
	{
		using namespace Utils::Math;

		for (auto& entity : Entities)
		{
			Eigen::Affine3f transform;
			transform = Eigen::Translation3f(entity.Position) * entity.Rotation.toRotationMatrix() * Eigen::Scaling(entity.Scale);
			entity.RenderModel.transform = ToMatrix(transform.matrix());

			DrawModel(entity.RenderModel, Vector3Zero(), 1.0f, entity.RenderColor);
		}

		PositionalConstraint.DrawConstraint();

		for (auto& forceInput : ForceInputs)
		{
			forceInput.Draw();
		}
	}

	void CubePositionalScene::OnDrawEditor()
	{
		Scene::OnDrawEditor();

		ImGui::Checkbox("Enable Gravity", &m_EnableGravity);
		if (m_EnableGravity)
		{
			ImGui::DragFloat("Gravity", &m_Gravity);
		}

		ImGui::SeparatorText("Entities");
		int i = 0;
		for (auto& entity : Entities)
		{
			i++;
			if (ImGui::TreeNode(TextFormat("Cube %d", i)))
			{
				m_IsDirty |= ImGui::Checkbox("Is Static", &entity.IsStaticBody);
				ImGui::DragFloat("Inverse Mass", &entity.InverseMass, 0.1f, 0.0f, 0.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp);

				ImGui::BeginDisabled(!Engine::Application::CheckUpdateMode(Engine::SimulationControls::UpdateMode::PAUSED));
				if (ImGui::DragFloat3("Reset Position", entity.ResetPosition.data()))
				{
					PositionalConstraint.TargetDistance = PositionalConstraint.Entity1->Position - PositionalConstraint.Entity2->Position;
					entity.Reset();
					m_IsDirty |= true;
				}
				ImGui::EndDisabled();

				ImGui::TreePop();
			}
		}

		ImGui::SeparatorText("Constraints");
		ImGui::DragFloat("Compliance", &PositionalConstraint.Compliance, 0.001f, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp);
		ImGui::DragFloat3("Target Distance", (PositionalConstraint.TargetDistance.data()));

		ImGui::SeparatorText("Force Input");
		for (auto& forceInput : ForceInputs)
		{
			m_IsDirty |= forceInput.DrawSettings();
		}
	}
}