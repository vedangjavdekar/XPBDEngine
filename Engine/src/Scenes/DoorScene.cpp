#include "DoorScene.h"
#include "Engine/Application.h"
#include "Constraints/HingeConstraint.h"
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
		static std::array<Simulation::Entity, 3> Entities;

		static std::array<Simulation::HingeConstraint, 1> HingeConstraint;
		static Simulation::PositionalConstraint PositionalConstraint;
		static std::array<Simulation::TransformationData, 2> TransformationData;

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

	DoorScene::DoorScene(const std::string& sceneName)
		: Scene(sceneName)
	{
		SetupEntities();
		SetupConstraints();
		SetupInputs();
	}

	DoorScene::~DoorScene()
	{
	}

	void DoorScene::OnInit()
	{
		for (auto& entity : Entities)
		{
			entity.Reset();
		}
	}

	void DoorScene::OnUpdate(const float substepTime)
	{
	}

	void DoorScene::OnStartSimulationFrame()
	{
		for (auto& entity : Entities)
		{
			entity.AddForce(
				Simulation::PhysicalForce{
					Eigen::Vector3f::Zero(),
					Eigen::Vector3f(0.0f, -10.0f * 1.0f / entity.InverseMass, 0.0f),
					false });
		}

		for (auto& forceInput : ForceInputs)
		{
			forceInput.Apply();
		}
	}

	void DoorScene::OnUpdatePosition(const float substepTime)
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

	void DoorScene::OnSolveConstraints(const float substepTime)
	{
		for (int i = 0; i < GetNumPosIterations(); ++i)
		{
			for (size_t j = 0; j < HingeConstraint.size(); ++j)
			{
				if (i == 0)
				{
					HingeConstraint[j].Init();
					TransformationData[j] = GetTransformationData(HingeConstraint[j].Entity1, HingeConstraint[j].Entity2);
				}

				ComputePositionalData(TransformationData[j], HingeConstraint[j].E1AttachPoint, HingeConstraint[j].E2AttachPoint);

				HingeConstraint[j].Solve(TransformationData[j], substepTime);
			}

			if (i == 0)
			{
				PositionalConstraint.Init();
				TransformationData[1] = GetTransformationData(PositionalConstraint.Entity1, PositionalConstraint.Entity2);
			}
			ComputePositionalData(TransformationData[1], PositionalConstraint.LocalR1, PositionalConstraint.LocalR2);

			PositionalConstraint.Solve(TransformationData[1], substepTime);

		}
	}

	void DoorScene::OnPostSolveConstraints(const float substepTime)
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

	void DoorScene::OnEndSimulationFrame()
	{
		for (auto& entity : Entities)
		{
			entity.Forces.clear();
		}
	}

	void DoorScene::OnShutdown()
	{
	}

	void DoorScene::SetupEntities()
	{
		const Eigen::Matrix3f inertiaTensor = ComputeInertiaTensorForCube(1.0f, 1.0f, 1.0f);
		const Eigen::Matrix3f invInertiaTensor = inertiaTensor.inverse();

		for (auto& entity : Entities)
		{
			entity.InverseMass = 1.0f;
			entity.RenderModel = Engine::Application::Get().GetResources().CubeModel;
			entity.InertiaTensor = inertiaTensor;
			entity.InverseInertiaTensor = invInertiaTensor;
		}

		Entities[0].RenderColor = YELLOW;
		Entities[0].IsStaticBody = true;
		Entities[0].ResetPosition = Eigen::Vector3f(0.0f, 1.5f, 0.0f);
		Entities[0].ResetScale = Eigen::Vector3f(0.25f, 2.0f, 0.25f);

		Entities[1].ResetPosition = Eigen::Vector3f(0.5f, 1.5f, 0.0f);
		Entities[1].ResetScale = Eigen::Vector3f(1.0f, 2.0f, 0.125f);

		Entities[2].RenderColor = GREEN;
		Entities[2].ResetPosition = Eigen::Vector3f(1.0f, 1.5f, 3.0f);
		Entities[2].ResetScale = Eigen::Vector3f(0.25f, 0.25f, 0.25f);
		Entities[2].IsStaticBody = true;
	}

	void DoorScene::SetupConstraints()
	{
		HingeConstraint[0].Compliance = 0.0f;
		HingeConstraint[0].Entity1 = &Entities[0];
		HingeConstraint[0].Entity2 = &Entities[1];

		HingeConstraint[0].E1AlignAxis = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
		HingeConstraint[0].E2AlignAxis = Eigen::Vector3f(0.0f, 1.0f, 0.0f);

		HingeConstraint[0].E1LimitAxis = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
		HingeConstraint[0].E2LimitAxis = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

		HingeConstraint[0].LimitAngle = true;
		HingeConstraint[0].LimitAngleMin = 0.0f;
		HingeConstraint[0].LimitAngleMax = 90.0f * DEG2RAD;

		HingeConstraint[0].E1AttachPoint = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
		HingeConstraint[0].E2AttachPoint = Eigen::Vector3f(-0.5f, 0.0f, 0.0f);

		PositionalConstraint.Entity1 = &Entities[2];
		PositionalConstraint.Entity2 = &Entities[1];
		PositionalConstraint.LocalR1 = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
		PositionalConstraint.LocalR2 = Eigen::Vector3f(0.5f, 0.0f, 0.0f);
		PositionalConstraint.TargetDistance = Entities[1].Position - Entities[2].Position;
		PositionalConstraint.Compliance = 0.5f;
	}

	void DoorScene::SetupInputs()
	{
		ForceInputs[0].Entity = &Entities[1];
		ForceInputs[0].ActivationKey = KEY_L;
		ForceInputs[0].ForcePosition = Eigen::Vector3f(0.5f, 0.5f, 0.0f);
		ForceInputs[0].IsLocal = true;
		ForceInputs[0].ForceVector = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

		ForceInputs[1].Entity = &Entities[1];
		ForceInputs[1].ActivationKey = KEY_K;
		ForceInputs[1].ForcePosition = Eigen::Vector3f(0.5f, -0.5f, 0.0f);
		ForceInputs[1].ForceVector = Eigen::Vector3f(0.0f, 0.0f, -10.0f);
		ForceInputs[1].IsRotationalForce = false;
		ForceInputs[1].IsLocal = true;
	}

	void DoorScene::OnDraw()
	{
		using namespace Utils::Math;

		for (auto& entity : Entities)
		{
			Eigen::Affine3f transform;
			transform = Eigen::Translation3f(entity.Position) * entity.Rotation.toRotationMatrix() * Eigen::Scaling(entity.Scale);
			entity.RenderModel.transform = ToMatrix(transform.matrix());

			DrawModel(entity.RenderModel, Vector3Zero(), 1.0f, entity.RenderColor);
		}

		for (auto& forceInput : ForceInputs)
		{
			forceInput.Draw();
		}

		HingeConstraint[0].DrawConstraint();
		PositionalConstraint.DrawConstraint();
	}

	void DoorScene::OnDrawEditor()
	{
		Scene::OnDrawEditor();

		ImGui::SeparatorText("Entities");
		int i = 0;
		for (auto& entity : Entities)
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

		for (size_t i = 0; i < HingeConstraint.size(); ++i)
		{
			if (ImGui::TreeNode(TextFormat("Constraint %d", i)))
			{
				ImGui::DragFloat("Compliance", &HingeConstraint[i].Compliance, 0.001f, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp);
				ImGui::Checkbox("LimitAngle", &HingeConstraint[i].LimitAngle);
				ImGui::SliderAngle("LimitAngleMin", &HingeConstraint[i].LimitAngleMin);
				ImGui::SliderAngle("LimitAngleMax", &HingeConstraint[i].LimitAngleMax);
				ImGui::TreePop();
			}
		}

		if (ImGui::TreeNode("Positional Constraint"))
		{
			ImGui::DragFloat("Compliance", &PositionalConstraint.Compliance, 0.001f, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp);
			ImGui::DragFloat3("TargetDistance", PositionalConstraint.TargetDistance.data());
			ImGui::TreePop();
		}



		ImGui::SeparatorText("Force Input");
		for (auto& forceInput : ForceInputs)
		{
			m_IsDirty |= forceInput.DrawSettings();
		}
	}
}