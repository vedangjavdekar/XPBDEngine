#include "CubeHingeScene.h"
#include "Engine/Application.h"
#include "Constraints/HingeConstraint.h"
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

		static std::array<Simulation::HingeConstraint, 2> HingeConstraint;
		static std::array <Simulation::TransformationData, 2> TransformationData;

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

	CubeHingeScene::CubeHingeScene(const std::string& sceneName)
		: Scene(sceneName)
	{
		SetupEntities();
		SetupConstraints();
		SetupInputs();
	}

	CubeHingeScene::~CubeHingeScene()
	{
	}

	void CubeHingeScene::OnInit()
	{
		for (auto& entity : Entities)
		{
			entity.Reset();
		}
	}

	void CubeHingeScene::OnUpdate(const float substepTime)
	{
	}

	void CubeHingeScene::OnStartSimulationFrame()
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

	void CubeHingeScene::OnUpdatePosition(const float substepTime)
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

	void CubeHingeScene::OnSolveConstraints(const float substepTime)
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
		}
	}

	void CubeHingeScene::OnPostSolveConstraints(const float substepTime)
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

	void CubeHingeScene::OnEndSimulationFrame()
	{
		for (auto& entity : Entities)
		{
			entity.Forces.clear();
		}
	}

	void CubeHingeScene::OnShutdown()
	{
	}

	void CubeHingeScene::SetupEntities()
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
		Entities[0].ResetPosition = Eigen::Vector3f(0.0f, 6.0f, 0.0f);
		Entities[0].ResetScale = Eigen::Vector3f(0.25f, 0.25f, 0.25f);

		Entities[1].ResetPosition = Eigen::Vector3f(0.0f, 4.25f, 0.0f);
		Entities[1].ResetScale = Eigen::Vector3f(0.25f, 3.0f, 0.25f);

		Entities[2].ResetPosition = Eigen::Vector3f(0.0f, 2.25f, 0.0f);
		Entities[2].ResetScale = Eigen::Vector3f(0.25f, 1.0f, 0.25f);
		Entities[2].RenderColor = GREEN;
	}

	void CubeHingeScene::SetupConstraints()
	{
		HingeConstraint[0].Compliance = 0.001f;
		HingeConstraint[0].Entity1 = &Entities[0];
		HingeConstraint[0].Entity2 = &Entities[1];

		HingeConstraint[0].E1AlignAxis = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
		HingeConstraint[0].E2AlignAxis = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

		HingeConstraint[0].E1LimitAxis = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
		HingeConstraint[0].E2LimitAxis = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

		HingeConstraint[0].E1AttachPoint = Eigen::Vector3f(0.0f, -0.25f, 0.0f);
		HingeConstraint[0].E2AttachPoint = Eigen::Vector3f(0.0f, 1.5f, 0.0f);

		HingeConstraint[1].Compliance = 0.001f;
		HingeConstraint[1].Entity1 = &Entities[1];
		HingeConstraint[1].Entity2 = &Entities[2];

		HingeConstraint[1].E1AlignAxis = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
		HingeConstraint[1].E2AlignAxis = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

		HingeConstraint[1].E1LimitAxis = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
		HingeConstraint[1].E2LimitAxis = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

		HingeConstraint[1].E1AttachPoint = Eigen::Vector3f(0.0f, -1.5f, 0.0f);
		HingeConstraint[1].E2AttachPoint = Eigen::Vector3f(0.0f, 0.5f, 0.0f);
	}

	void CubeHingeScene::SetupInputs()
	{
		ForceInputs[0].Entity = &Entities[1];
		ForceInputs[0].ActivationKey = KEY_L;
		ForceInputs[0].ForceVector = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
		ForceInputs[0].ForcePosition = Eigen::Vector3f(-1.0f, 0.0f, 0.0f);
		ForceInputs[0].IsLocal = true;

		ForceInputs[1].Entity = &Entities[1];
		ForceInputs[1].ActivationKey = KEY_K;
		ForceInputs[1].ForcePosition = Eigen::Vector3f(0.5f, 0.0f, 0.0f);
		ForceInputs[1].ForceVector = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
		ForceInputs[1].IsRotationalForce = false;
		ForceInputs[1].IsLocal = true;
	}

	void CubeHingeScene::OnDraw()
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
	}

	void CubeHingeScene::OnDrawEditor()
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
				ImGui::TreePop();
			}
		}

		ImGui::SeparatorText("Force Input");
		for (auto& forceInput : ForceInputs)
		{
			m_IsDirty |= forceInput.DrawSettings();
		}
	}
}