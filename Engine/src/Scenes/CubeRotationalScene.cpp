#include "CubeRotationalScene.h"
#include "Engine/Application.h"
#include "Constraints/RotationalConstraint.h"
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

		static Simulation::RotationalConstraint RotationalConstraint;
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

	CubeRotationalScene::CubeRotationalScene(const std::string &sceneName)
		: Scene(sceneName)
	{
		SetupEntites();
		SetupConstraints();
		SetupInputs();
	}

	CubeRotationalScene::~CubeRotationalScene()
	{
	}

	void CubeRotationalScene::OnInit()
	{
		for (auto &entity : Entities)
		{
			entity.Reset();
		}
	}

	void CubeRotationalScene::OnUpdate(const float substepTime)
	{
	}

	void CubeRotationalScene::OnStartSimulationFrame()
	{
		for (auto &forceInput : ForceInputs)
		{
			forceInput.Apply();
		}
	}

	void CubeRotationalScene::OnUpdatePosition(const float substepTime)
	{
		for (auto &entity : Entities)
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
				entity.AngularVelocity(2)};

			Eigen::Quaternionf wq = OmegaQuaternion * entity.Rotation;
			wq.coeffs() = entity.Rotation.coeffs() + substepTime * 0.5f * wq.coeffs();

			entity.Rotation = wq.normalized();

			Engine::DebugDrawing::DrawForceMarker(BLUE, entity.Position, entity.Rotation, entity.Position, totalForce, false, -1.0f, 0.0f);
		}
	}

	void CubeRotationalScene::OnSolveConstraints(const float substepTime)
	{
		for (int i = 0; i < GetNumPosIterations(); ++i)
		{
			if (i == 0)
			{
				RotationalConstraint.Lambda = 0.0f;
				TransformationData = GetTransformationData(RotationalConstraint.Entity1, RotationalConstraint.Entity2);
			}

			const Eigen::Vector3f world1X = Entities[0].Rotation.toRotationMatrix() * Eigen::Vector3f(1.0f, 0.0f, 0.0f);
			const Eigen::Vector3f world2X = Entities[1].Rotation.toRotationMatrix() * Eigen::Vector3f(1.0f, 0.0f, 0.0f);
			const Eigen::Vector3f deltaQ = world1X.cross(world2X);


			RotationalConstraint.Solve(TransformationData, substepTime, deltaQ);
		}
	}

	void CubeRotationalScene::OnPostSolveConstraints(const float substepTime)
	{
		for (auto &entity : Entities)
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

	void CubeRotationalScene::OnEndSimulationFrame()
	{
		for (auto &entity : Entities)
		{
			entity.Forces.clear();
		}
	}

	void CubeRotationalScene::OnShutdown()
	{
	}

	void CubeRotationalScene::SetupEntites()
	{
		Entities[0].RenderModel = Engine::Application::Get().GetResources().CubeModel;
		Entities[1].RenderModel = Engine::Application::Get().GetResources().CubeModel;

		Entities[0].InverseMass = 1.0f;
		Entities[0].InertiaTensor = ComputeInertiaTensorForCube(1.0f, 1.0f, 1.0f);
		Entities[0].InverseInertiaTensor = Entities[0].InertiaTensor.inverse();
		Entities[0].ResetPosition = Eigen::Vector3f(-1.0f, 2.0f, 0.0f);
		Entities[0].IsStaticForCorrection = true;

		Entities[1].InverseMass = 1.0f;
		Entities[1].InertiaTensor = ComputeInertiaTensorForCube(0.1f, 0.1f, 0.1f);
		Entities[1].InverseInertiaTensor = Entities[1].InertiaTensor.inverse();
		Entities[1].ResetPosition = Eigen::Vector3f(1.0f, 2.0f, 0.0f);
	}

	void CubeRotationalScene::SetupConstraints()
	{
		RotationalConstraint.Compliance = 0.001f;
		RotationalConstraint.Entity1 = &Entities[0];
		RotationalConstraint.Entity2 = &Entities[1];
	}

	void CubeRotationalScene::SetupInputs()
	{
		ForceInputs[0].Entity = &Entities[0];
		ForceInputs[0].ActivationKey = KEY_L;
		ForceInputs[0].ForcePosition = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
		ForceInputs[0].IsLocal = true;
		ForceInputs[0].ForceVector = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
		ForceInputs[0].IsRotationalForce = true;

		ForceInputs[1].Entity = &Entities[1];
		ForceInputs[1].ActivationKey = KEY_K;
		ForceInputs[1].ForcePosition = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
		ForceInputs[1].IsLocal = true;
		ForceInputs[1].ForceVector = Eigen::Vector3f(0.0f, -1.0f, 0.0f);
		ForceInputs[1].IsRotationalForce = true;
	}

	void CubeRotationalScene::OnDraw()
	{
		using namespace Utils::Math;

		for (auto &entity : Entities)
		{
			Eigen::Affine3f transform;
			transform = Eigen::Translation3f(entity.Position) * entity.Rotation.toRotationMatrix() * Eigen::Scaling(entity.Scale);
			entity.RenderModel.transform = ToMatrix(transform.matrix());

			DrawModel(entity.RenderModel, Vector3Zero(), 1.0f, entity.RenderColor);
		}

		for (auto &forceInput : ForceInputs)
		{
			forceInput.Draw();
		}
	}

	void CubeRotationalScene::OnDrawEditor()
	{
		Scene::OnDrawEditor();

		ImGui::SeparatorText("Entities");
		int i = 0;
		for (auto &entity : Entities)
		{
			i++;
			if (ImGui::TreeNode(TextFormat("Cube %d", i)))
			{
				m_IsDirty |= ImGui::Checkbox("Is Static", &entity.IsStaticBody);
				ImGui::DragFloat("Inverse Mass", &entity.InverseMass, 0.1f, 0.0f, 0.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp);

				ImGui::TreePop();
			}
		}

		ImGui::SeparatorText("Constraints");
		ImGui::DragFloat("Compliance", &RotationalConstraint.Compliance, 0.001f, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp);

		ImGui::SeparatorText("Force Input");
		for (auto &forceInput : ForceInputs)
		{
			m_IsDirty |= forceInput.DrawSettings();
		}
	}
}