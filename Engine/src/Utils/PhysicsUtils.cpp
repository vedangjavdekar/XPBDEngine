#include "PhysicsUtils.h"
#include "Engine/DebugDrawing.h"
#include "Engine/Entity.h"

#include "Utils/EigenToRaylib.h"
#include "imgui.h"

namespace Utils::Physics
{
	void ForceInput::ProcessEvents()
	{
		if (SkipProcessInputsNextFrame)
		{
			SkipProcessInputsNextFrame = false;
			return;
		}

		switch (ActivationMode)
		{
		case Utils::Physics::Toggle:
		{
			if (IsKeyPressed(ActivationKey))
			{
				IsActive = !IsActive;
			}
			break;
		}
		case Utils::Physics::Pressed:
		{
			IsActive = IsKeyDown(ActivationKey);
			break;
		}
		default:
			break;
		}
	}

	void ForceInput::Apply()
	{
		ProcessEvents();
		if (!Entity || !IsActive)
		{
			return;
		}

		Engine::DebugDrawing::DrawForceMarker(YELLOW, Entity->Position, Entity->Rotation, ForcePosition, ForceVector, IsLocal, -1.0f, 0.0f);
		Simulation::PhysicalForce result = Simulation::PhysicalForce{ ForcePosition,ForceVector,IsLocal };
		Entity->AddForce(result);

		if (IsRotationalForce)
		{
			Eigen::Vector3f reflectedPoint = -ForcePosition;
			Eigen::Vector3f forceVector = -ForceVector;
			Simulation::PhysicalForce result2{ reflectedPoint, forceVector, IsLocal };
			Entity->AddForce(result2);
			Engine::DebugDrawing::DrawForceMarker(ORANGE, Entity->Position, Entity->Rotation, reflectedPoint, forceVector, IsLocal, -1.0f, 0.0f);
		}
	}

	void ForceInput::Draw()
	{
		using namespace Utils::Math;

		if (!Entity)
		{
			return;
		}

		if (ForceVector.isZero())
		{
			return;
		}

		Eigen::Vector3f forcePosition = ForcePosition;
		Eigen::Vector3f forceVector = ForceVector;
		if (IsLocal)
		{
			forcePosition = Entity->Position + Entity->Rotation.toRotationMatrix() * ForcePosition;
			forceVector = Entity->Rotation.toRotationMatrix() * ForceVector;
		}

		DrawCylinderEx(ToVector3(forcePosition - MarkerScale * forceVector.normalized()), ToVector3(forcePosition), 0.5f * MarkerScale, 0.0f, 8, GOLD);

		if (IsRotationalForce)
		{
			Eigen::Vector3f forcePosition2 = -ForcePosition;
			Eigen::Vector3f forceVector2 = -ForceVector;
			if (IsLocal)
			{
				forcePosition = Entity->Position + Entity->Rotation.toRotationMatrix() * forcePosition2;
				forceVector = Entity->Rotation.toRotationMatrix() * forceVector2;
			}

			DrawCylinderEx(ToVector3(forcePosition - MarkerScale * forceVector.normalized()), ToVector3(forcePosition), 0.5f * MarkerScale, 0.0f, 8, ORANGE);
		}
	}

	bool ForceInput::DrawSettings()
	{
		bool result = false;

		if (ImGui::TreeNode(TextFormat("Key: %c##%u", ActivationKey, this)))
		{
			if (ImGui::Checkbox("Is Active", &IsActive))
			{
				SkipProcessInputsNextFrame = true;
				result |= true;
			}
			result |= ImGui::DragFloat("Marker Scale", &MarkerScale);
			result |= ImGui::DragFloat3("Force Direction", ForceVector.data());
			result |= ImGui::DragFloat3("Force Position", ForcePosition.data());
			result |= ImGui::Checkbox("Is Local", &IsLocal);
			ImGui::TreePop();
		}

		return result;
	}
}